
#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

//---------------- CONFIGURACOES ----------------//

#define BAUD 19200UL
#define MYUBRR (F_CPU/16/BAUD-1)

#define MY_TEMPO 5
#define INTERV 60

#define QTD_LEITURAS 40
#define QTD_AMOSTRAS (INTERV/MY_TEMPO)

#define TIMER1_PRELOAD 0xC2F6
#define RAD_TO_DEG 57.2957795131

// Flags do sistema
#define FLAG_TIMER (1 << 0)
#define FLAG_ADC   (1 << 1)

// Buffer circular USART
#define RX_BUFFER_SIZE 32
#define RX_BUFFER_MASK (RX_BUFFER_SIZE-1)

// Pino da bomba
#define BOMBA PB4

//---------------- VARIAVEIS GLOBAIS ----------------//

volatile uint16_t ReadSensor[QTD_LEITURAS];

uint16_t mdReadSensor = 0;

volatile uint16_t vector_mdRead[QTD_AMOSTRAS];

volatile uint8_t idex = 0;
volatile uint8_t t_read = 0;
volatile uint8_t current_time = 0;

volatile uint8_t FLAGData = 0;

long int size_tank = 1023;
long int mdReadSensor_L = 0;

double angle = 0;

// Buffer circular USART
volatile uint8_t rx_buffer[RX_BUFFER_SIZE];

volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;


//-------------- INTERRUPCAO TIMER1 ----------------//

ISR(TIMER1_OVF_vect)
{
	TCNT1 = TIMER1_PRELOAD;

	PORTB ^= (1 << PB5);

	t_read++;

	if(t_read >= MY_TEMPO)
	{
		t_read = 0;

		current_time++;

		FLAGData |= FLAG_TIMER;
	}
}


//-------------- INTERRUPCAO ADC ----------------//

ISR(ADC_vect)
{
	ReadSensor[idex] = ADC;

	idex++;

	if(idex >= QTD_LEITURAS)
	{
		idex = 0;

		FLAGData |= FLAG_ADC;

		// Desabilita o ADC após 40 conversões
		ADCSRA &= ~(1 << ADEN);
	}
}


//-------------- INTERRUPCAO USART ----------------//

ISR(USART_RX_vect)
{
	uint8_t dado = UDR0;

	uint8_t next = (rx_head + 1) & RX_BUFFER_MASK;

	if(next != rx_tail)
	{
		rx_buffer[rx_head] = dado;

		rx_head = next;
	}
}


//-------------- CONFIGURACAO ADC ----------------//

void initADC(void)
{
	// Referência AVcc e canal ADC0
	ADMUX = (1 << REFS0);

	// Habilita ADC, conversão, auto-trigger,
	// interrupção e prescaler 128
	ADCSRA = (1 << ADEN)  |
	(1 << ADSC)  |
	(1 << ADATE) |
	(1 << ADIF)  |
	(1 << ADIE)  |
	(1 << ADPS2) |
	(1 << ADPS1) |
	(1 << ADPS0);

	// Free-running
	ADCSRB = 0x00;

	// Desabilita entradas digitais ADC1 até ADC7
	DIDR0 = 0b11111110;
}


//-------------- CONFIGURACAO TIMER1 ----------------//

void initTimer1(void)
{
	TCNT1 = TIMER1_PRELOAD;

	TIFR1 = (1 << TOV1);

	// Modo normal
	TCCR1A = 0x00;

	// Prescaler 1024
	TCCR1B = 0x05;

	// Habilita interrupção de overflow
	TIMSK1 |= (1 << TOIE1);
}


//-------------- CONFIGURACAO USART ----------------//

void USART_Init(unsigned int ubrr)
{
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;

	// Habilita interrupção de recepção,
	// receptor e transmissor
	UCSR0B = (1 << RXCIE0) |
	(1 << RXEN0)  |
	(1 << TXEN0);

	// Comunicação 8N2
	UCSR0C = (1 << USBS0)  |
	(1 << UCSZ01) |
	(1 << UCSZ00);
}


//-------------- CALCULO DA MEDIA ----------------//

uint16_t MD_ReadSensor(volatile uint16_t vector[], uint8_t sizeData)
{
	int32_t soma = 0;

	if(sizeData == 0)
	{
		return 0;
	}

	for(uint8_t i = 0; i < sizeData; i++)
	{
		soma += vector[i];
	}

	return (uint16_t)(soma / sizeData);
}


//-------------- CALCULO DA INCLINACAO ----------------//

void CalculateSlope(void)
{
	double delta;
	double slope;

	// Diferença entre última e primeira média
	delta = (double)vector_mdRead[QTD_AMOSTRAS-1]
	- (double)vector_mdRead[0];

	// Intervalo entre primeira e última amostra: 55 segundos
	slope = delta / ((QTD_AMOSTRAS - 1) * MY_TEMPO);

	// Conversão da inclinação para graus
	angle = atan(slope) * RAD_TO_DEG;
}


//-------------- TRANSMISSAO USART ----------------//

void USART_Transmit(const uint8_t data[], uint16_t sizeData)
{
	for(uint16_t i = 0; i < sizeData; i++)
	{
		while(!(UCSR0A & (1 << UDRE0)));

		UDR0 = data[i];
	}
}


void USART_TransmitString(const char *mensagem)
{
	USART_Transmit((const uint8_t *)mensagem, strlen(mensagem));
}


//-------------- RECEPCAO USART ----------------//

// Retorno:
//  1 = número recebido corretamente
//  0 = aguardando mensagem completa
// -1 = mensagem inválida ou muito grande

int8_t USART_Receive(long int *valor)
{
	static char mensagem[12];

	static uint8_t indice = 0;

	static uint8_t overflow = 0;

	while(rx_tail != rx_head)
	{
		uint8_t dado = rx_buffer[rx_tail];

		rx_tail = (rx_tail + 1) & RX_BUFFER_MASK;

		// Ecoa caractere recebido
		USART_Transmit(&dado, 1);

		// Verifica fim da mensagem
		if(dado == '\r' || dado == '\n')
		{
			// Ignora mensagens vazias
			if(indice == 0 && !overflow)
			{
				continue;
			}

			// Verifica excesso de caracteres
			if(overflow)
			{
				indice = 0;
				overflow = 0;

				USART_TransmitString("\nENTRADA MUITO GRANDE!\n");

				return -1;
			}

			// Finaliza string
			mensagem[indice] = '\0';

			char *fim;

			errno = 0;

			// Converte string para inteiro
			long int numero = strtol(mensagem, &fim, 10);

			// Verifica erros de conversão
			if(fim == mensagem || *fim != '\0' || errno == ERANGE)
			{
				indice = 0;

				USART_TransmitString("\nENTRADA INVALIDA!\n");

				return -1;
			}

			*valor = numero;

			indice = 0;

			return 1;
		}

		// Armazena caracteres recebidos
		if(!overflow)
		{
			if(indice < sizeof(mensagem) - 1)
			{
				mensagem[indice++] = dado;
			}
			else
			{
				overflow = 1;
			}
		}
	}

	return 0;
}


//-------------- PROGRAMA PRINCIPAL ----------------//

int main(void)
{
	char saida[20];

	// 1 = bomba habilitada
	// 0 = bomba desabilitada
	long int KEYBOMBA = 1;

	long int recebido = 0;

	int8_t status;

	uint8_t bomba_ligar = 0;

	uint8_t flags;

	double nivel;

	//---------------- CONFIGURACAO DOS PINOS ----------------//

	// PB4 = saída da bomba
	// PB5 = sinalização
	DDRB |= (1 << PB4) | (1 << PB5);

	// PORTC como entrada
	DDRC = 0x00;

	// Pull-ups em PC1 até PC5
	// PC0 permanece sem pull-up para o ADC
	PORTC = 0x3E;

	// Inicializa PORTB
	PORTB = 0x00;


	//---------------- INICIALIZACAO DOS PERIFERICOS ----------------//

	initADC();

	initTimer1();

	USART_Init(MYUBRR);

	sei();


	USART_TransmitString(
	"--------------------------------------------------------------------\n"
	"\t\tINICIALIZANDO O SISTEMA\n"
	"--------------------------------------------------------------------\n"
	);

	USART_TransmitString("TEMPO DE RESPOSTA = 1 min\n");

	USART_TransmitString("INSIRA O VOLUME DO SEU RESERVATORIO(L): ");


	//---------------- RECEPCAO DO VOLUME ----------------//

	do
	{
		status = USART_Receive(&recebido);

		if(status == -1)
		{
			USART_TransmitString("INSIRA NOVAMENTE: ");
		}

		else if(status == 1 && recebido <= 0)
		{
			USART_TransmitString(
			"\nVOLUME INVALIDO!\nINSIRA NOVAMENTE: "
			);

			status = 0;
		}

	} while(status != 1);


	size_tank = recebido;

	USART_TransmitString(
	"\nENVIE 0 PARA DESLIGAR A BOMBA OU 1 PARA HABILITA-LA.\n"
	);


	//---------------- LOOP PRINCIPAL ----------------//

	while(1)
	{
		// Captura e limpa as flags atomicamente.
		// Evita perder eventos gerados pelas interrupções.
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			flags = FLAGData;
			FLAGData = 0;
		}


		//---------------- PROCESSAMENTO ADC ----------------//

		if(flags & FLAG_ADC)
		{
			// Calcula média das 40 leituras
			mdReadSensor = MD_ReadSensor(
			ReadSensor,
			QTD_LEITURAS
			);
		}


		//---------------- PROCESSAMENTO TIMER ----------------//

		if(flags & FLAG_TIMER)
		{
			// Armazena média na posição correspondente
			if(current_time > 0 && current_time <= QTD_AMOSTRAS)
			{
				vector_mdRead[current_time - 1] = mdReadSensor;
			}


			// Verifica conclusão dos 60 segundos
			if(current_time >= QTD_AMOSTRAS)
			{
				current_time = 0;

				// Calcula inclinação do nível
				CalculateSlope();

				// Converte leitura ADC para litros
				mdReadSensor_L =
				((int64_t)mdReadSensor * size_tank) / 1023;


				//---------------- ANALISE DO CONSUMO ----------------//

				if((-90 < angle) && (angle <= -45))
				{
					USART_TransmitString(
					"\nCONSUMO EXORBITANTE DETECTADO\n"
					);

					KEYBOMBA = 0;

					bomba_ligar = 0;
				}

				else if((-45 < angle) && (angle < 0))
				{
					USART_TransmitString(
					"\nCONSUMO DETECTADO\n"
					);
				}

				else if((0 <= angle) && (angle <= 90))
				{
					USART_TransmitString(
					"\nSEM CONSUMO \n"
					);
				}


				//---------------- CONTROLE DA BOMBA ----------------//

				if(KEYBOMBA == 1)
				{
					nivel = (double)mdReadSensor / 1023.0;

					// Nível crítico: abaixo ou igual a 5%
					if(nivel <= 0.05)
					{
						bomba_ligar = 0;

						KEYBOMBA = 0;

						USART_TransmitString(
						"\nPROBLEMA ESTRUTURAL\n"
						);
					}

					// Abaixo de 90%: liga a bomba
					else if(nivel < 0.90)
					{
						bomba_ligar = 1;
					}

					// Entre 90% e 95%: mantém estado anterior
					else if(nivel < 0.95)
					{
						// Não altera bomba_ligar
					}

					// A partir de 95%: desliga a bomba
					else
					{
						bomba_ligar = 0;
					}
				}

				else
				{
					// Se a bomba estiver desabilitada,
					// garante que permaneça desligada
					bomba_ligar = 0;
				}


				//---------------- ACIONAMENTO FISICO ----------------//

				if(KEYBOMBA == 1 && bomba_ligar == 1)
				{
					PORTB |= (1 << BOMBA);

					USART_TransmitString("\nBOMBA LIGADA\n");
				}
				else
				{
					PORTB &= ~(1 << BOMBA);

					USART_TransmitString("\nBOMBA DESLIGADA\n");
				}


				//---------------- TRANSMISSAO DO NIVEL ----------------//

				USART_TransmitString("NIVEL RESERVATORIO(L) = ");

				snprintf(
				saida,
				sizeof(saida),
				"%ld \n",
				mdReadSensor_L
				);

				USART_TransmitString(saida);
			}


			//---------------- REINICIALIZACAO ADC ----------------//

			// Habilita novamente o ADC e inicia nova sequência
			ADCSRA |= (1 << ADEN) | (1 << ADSC);
		}


		//---------------- RECEPCAO DE COMANDOS USART ----------------//

		status = USART_Receive(&recebido);

		if(status == 1)
		{
			// Aceita somente 0 ou 1
			if(recebido == 0 || recebido == 1)
			{
				KEYBOMBA = recebido;

				if(KEYBOMBA == 0)
				{
					// Desliga imediatamente a bomba
					bomba_ligar = 0;

					PORTB &= ~(1 << BOMBA);

					USART_TransmitString(
					"\nBOMBA DESABILITADA PELO USUARIO\n"
					);
				}

				else
				{
					USART_TransmitString(
					"\nBOMBA HABILITADA PELO USUARIO\n"
					);
				}
			}

			else
			{
				USART_TransmitString(
				"\nNUMERO INVALIDO! ENVIE 0 OU 1.\n"
				);
			}
		}
	}

	return 0;
}
