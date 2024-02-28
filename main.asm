; Projeto Relógio Digital
; Disciplina: Microcontroladores e Aplicações
; Professor: Dr. Erick de Andrade Barboza
; Alunos:
;  - Robson Bruno Alves Pinheiro
;  - Victor Alexandre da R. Monteiro Miranda
; Descrição: Implementação de um relógio digital usando o microcontrolador AVR ATmega328P.

; Definição do vetor de interrupções
jmp reset
.org OC1Aaddr
jmp TIMER1_OVF_ISR

; Inicialização do programa
reset:

.equ DELAY_COUNT = 1000     ; Contador de atraso (1 segundo)

.equ DIGIT_PORT = PORTB     ; Porta usada para selecionar o dígito
.equ SEGMENT_PORT = PORTD   ; Porta usada para controlar os segmentos

.equ SEG_A = 0b01111111    ; Segmentos de exibição de 7 segmentos (cátodo comum)
.equ SEG_B = 0b10111111
.equ SEG_C = 0b11011111
.equ SEG_D = 0b11101111
.equ SEG_E = 0b11110111
.equ SEG_F = 0b11111011
.equ SEG_G = 0b11111101
.equ SEG_DP = 0b11111110

; Tabela de conversão de dígito para segmentos (0 a 9)
.equ SEGMENT_TABLE = SEG_TABLE_END - SEG_TABLE_START

SEG_TABLE_START:
    .db SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G
SEG_TABLE_END:

; Pinos
.equ pin_mode = PINB0    ; Pino para o botão MODE
.equ pin_start = PINB1   ; Pino para o botão START
.equ pin_reset = PINB2   ; Pino para o botão RESET

; Definições de variáveis como registradores
.def temp_secs_counter = r16   ; Registrador para armazenar os segundos (cronômetro)
.def temp_minutes_counter = r17   ; Registrador para armazenar os minutos (cronômetro)
.def state = r18          ; Registrador para armazenar o state atual
.def flag_start = r19      ; Registrador para a flag do botão START
.def flag_reset = r20      ; Registrador para a flag do botão RESET
.def flag_mode = r21       ; Registrador para a flag do botão MODE
.def minutes_counter = r22          ; Registrador para armazenar o minuto
.def secs_counter = r23         ; Registrador para armazenar o segundo
.def string_pointer = r24       ; Registrador para armazenar o ponteiro da string
.def string_pointer2 = r25      ; Registrador para armazenar o ponteiro da string
.def config_state = r26           ; Registrador para armazenar o estado atual do modo de configuração
.def config_minutes_counter = r27 ; Registrador para armazenar os segundos (configuração)
.def config_secs_counter = r28  ; Registrador para armazenar os minutos (configuração)
.def temp = r29            ; Registrador temporário
.def temp_seg = r30            ; Registrador temporário 2

; Configuração UART
; 16Mhz, 9600 baud, UBRR = 103
.equ UBRRvalue = 103
ldi temp, high (UBRRvalue) ; baud rate
sts UBRR0H, temp
ldi temp, low (UBRRvalue)
sts UBRR0L, temp
; 8 bits, 1 bit de parada, sem paridade
ldi temp, (3<<UCSZ00)
sts UCSR0C, temp
ldi temp, (1<<TXEN0) ; habilitando transmissão
sts UCSR0B, temp;

; Definição de constantes que serão enviadas via serial
mode1_string: .db "[MODO 1] ", 0
mode2_string: .db "[MODO 2] ", 0
mode3_string: .db "[MODO 3] ", 0
zero_string: .db " ZERO", 0
start_string: .db " START", 0
reset_string: .db " RESET", 0
us_string: .db " Ajustando a unidade dos segundos", 0
ds_string: .db " Ajustando a dezena dos segundos", 0
um_string: .db " Ajustando a unidade dos minutos", 0
dm_string: .db " Ajustando a dezena dos minutos", 0

; Configuração de interrupções
#define DELAY 1 ;segundos
.equ clk = 16.0e6 ;clock speed
.equ PRESCALE = 0b100 ;/256 prescale
.equ PRESCALE_DIV = 256
.equ WGM = 0b0100 ;Waveform generation mode: CTC
.equ TOP = int(0.5 + ((clk/PRESCALE_DIV)*DELAY))
.if TOP > 65535
.error "TOP is out of range"
.endif

ldi temp, high(TOP) ;initialize compare value (TOP)
sts OCR1AH, temp
ldi temp, low(TOP)
sts OCR1AL, temp
ldi temp, ((WGM&0b11) << WGM10) ;lower 2 bits of WGM
; WGM&0b11 = 0b0100 & 0b0011 = 0b0000
sts TCCR1A, temp
;upper 2 bits of WGM and clock select
ldi temp, ((WGM>> 2) << WGM12)|(PRESCALE << CS10)
; WGM >> 2 = 0b0100 >> 2 = 0b0001
; (WGM >> 2) << WGM12 = (0b0001 << 3) = 0b0001000
; (PRESCALE << CS10) = 0b100 << 0 = 0b100
; 0b0001000 | 0b100 = 0b0001100
sts TCCR1B, temp ;start counter
lds temp, TIMSK1
sbr temp, 1 <<OCIE1A
sts TIMSK1, temp
sei
; Fim da configuração de interrupções


; Configuração de inicialização segmentos
ldi temp, 0xFF         ; Configura todas as portas como saída
out DDRB, temp
out DDRD, temp
ldi temp, 0       ; Inicializa os dígitos como desligados
out DIGIT_PORT, temp
; Fim da Configuração de inicialização segmentos

;Inicialização da stack
ldi temp, low(RAMEND)
out SPL, temp
ldi temp, high(RAMEND)
out SPH, temp

; configuração de portas
ldi temp, (1<<pin_mode)|(1<<pin_start)|(1<<pin_reset) ; Configura pinos 
out DDRB, temp                   

; configuração inicial dos registradores
clr temp_secs_counter
clr temp_minutes_counter
clr secs_counter
clr minutes_counter
clr state ; Inicializa o estado do sistema como 0 (modo relógio)

; Loop principal do programa
loop:
    ;limpa flags imediatamente após trocar de modo
    clr flag_start
    clr flag_reset

    mov temp, state ; Carrega o estado atual de funcionamento em temp

    ; Compara temp com os valores definidos para cada modo
    cpi temp, 0x0
    breq modo_relogio
    cpi temp, 0x01
    breq modo_cronometro
    cpi temp, 0x02
    breq modo_configuracao

    rjmp loop


;(MODO 1 DE OPERAÇÃO)
; Enquanto MODE não for pressionado, o sistema permanece no modo de relógio
modo_relogio:
    call atualiza_display

    ; Verifica botão MODE
    call poll_mode

    rjmp loop

;(MODO 2 DE OPERAÇÃO)
; Enquanto MODE não for pressionado, o sistema permanece no modo de cronômetro
modo_cronometro:
    call atualiza_display

    ; Verifica botão MODE
    call poll_mode

    ; Verifica botão START
    call poll_start

    ; Verifica botão RESET
    call poll_reset

    rjmp loop

;(MODO 3 DE OPERAÇÃO)
; Enquanto MODE não for pressionado, o sistema permanece no modo de configuração
modo_configuracao:
    call atualiza_display

    ; Verifica botão MODE
    call poll_mode

    ; Verifica botão START
    call poll_start

    ; Verifica botão RESET
    call poll_reset

    rjmp loop

; Rotina: inicia_para_tempo
; Descrição: Inicia ou para o cronômetro de acordo com flag_start
; Parâmetros: temp_minutes_counter, temp_secs_counter, flag_start
inicia_para_tempo:
    call uart_modo_cronometro_start
    push temp
    ldi temp, 0x01
    eor flag_start, temp
    pop temp
    ret

; Rotina: reinicia_tempo
; Descrição: Reinicia o tempo do cronômetro
; Parâmetros: temp_minutes_counter, temp_secs_counter
reinicia_tempo:
    cpi flag_start, 0; Se a flag não estiver 1, pular rotina
    brne fim_reinicia_tempo

    clr temp_minutes_counter
    clr temp_secs_counter

    call atualiza_display
    call uart_modo_cronometro_reset

fim_reinicia_tempo:
    ret

; Rotina: ajusta_tempo
; Descrição: Alterna entre os possíveis modos: Ajustes de unidade dos segundos, dezena dos segundos, unidade dos minutos e dezena dos minutos
ajusta_tempo:
    inc config_state
    cpi config_state, 0x03
    brne fim_ajusta_tempo
    clr config_state

fim_ajusta_tempo:
    ret

; Rotina: aplica_ajuste
; Descrição: Aplica o ajuste feito no modo de configuração
; Parâmetros: minutos_counter, config_minutes_counter, secs_counter, config_secs_counter
aplica_ajuste:
    mov minutes_counter, config_minutes_counter
    mov secs_counter, config_secs_counter
    ret

; Rotina: atualiza_display
; Descrição: Atualiza o display de acordo com o estado atual do sistema
; Parâmetros: state, estado atual do sistema
atualiza_display:
	push temp
    cpi state, 0x00
	breq atualiza_relogio
	cpi state, 0x01
	breq atualiza_cronometro
	cpi state, 0x02
	breq atualiza_config
    rjmp fim_atualiza_display

; Atualiza o display com os valores do modo relógio
atualiza_relogio:
	mov temp, secs_counter
    call DISPLAY_TWO_DIGITS
    mov temp, minutes_counter
    call DISPLAY_TWO_DIGITS
	rjmp fim_atualiza_display

; Atualiza o display com os valores do modo cronômetro
atualiza_cronometro:
	mov temp, temp_secs_counter
    call DISPLAY_TWO_DIGITS
    mov temp, temp_minutes_counter
    call DISPLAY_TWO_DIGITS
	rjmp fim_atualiza_display

; Atualiza o display com os valores do modo configuração
atualiza_config:
	mov temp, config_secs_counter
    call DISPLAY_TWO_DIGITS
    mov temp, config_minutes_counter
    call DISPLAY_TWO_DIGITS
	rjmp fim_atualiza_display

fim_atualiza_display:
	pop temp
    ret

; Rotina: uart_modo_relogio
; Descrição: Envia "[MODO 1] MM:SS" via serial
; Parâmetros: Nada.
uart_modo_relogio:
    ; Envia "[MODO 1]"
    ldi r24, low(mode1_string)
    ldi r25, high(mode1_string)
    call send_string

    ; Envia "MM:SS"
    call to_ascii
    call send_char
    ldi temp, ':'
    call send_char
    mov temp, temp_seg
    call send_char

    ret

; Rotina: uart_modo_cronometro_start
; Descrição: Envia "[MODO 2] " via serial, chamada por outras rotinas de uart do modo cronômetro
; Parâmetros: Nada.
uart_modo_cronometro:
    ; Envia "[MODO 2]"
    ldi r24, low(mode2_string)
    ldi r25, high(mode2_string)
    call send_string

    ret

; Rotina: uart_modo_cronometro_zero
; Descrição: Envia "[MODO 2] ZERO" via serial
; Parâmetros: Nada.
uart_modo_cronometro_zero:
    call uart_modo_cronometro ; Envia "[MODO 2]"

    ; Envia " ZERO"
    ldi r24, low(zero_string)
    ldi r25, high(zero_string)
    call send_string

    ret

; Rotina: uart_modo_cronometro_start
; Descrição: Envia "[MODO 2] START" via serial
; Parâmetros: Nada.
uart_modo_cronometro_start:
    call uart_modo_cronometro ; Envia "[MODO 2]"

    ; Envia " START"
    ldi r24, low(start_string)
    ldi r25, high(start_string)
    call send_string

    ret

; Rotina: uart_modo_cronometro_start
; Descrição: Envia "[MODO 2] RESET" via serial
; Parâmetros: Nada.
uart_modo_cronometro_reset:
    call uart_modo_cronometro ; Envia "[MODO 2]"

    ; Envia " RESET"
    ldi r24, low(reset_string)
    ldi r25, high(reset_string)
    call send_string

    ret

; Rotina: uart_modo_configuracao
; Descrição: Envia mensagens de configuração via serial de acordo com o estado atual de config_state
; Parâmetros: config_state, estado atual do modo de configuração
uart_modo_configuracao:
    ; Envia "[MODO 3]"
    ldi r24, low(mode3_string)
    ldi r25, high(mode3_string)
    call send_string

    ; Lógica para imprimir na serial de acordo com o estado atual de config_state
    cpi config_state, 0x00
    breq envia_us
    cpi config_state, 0x01
    breq envia_ds
    cpi config_state, 0x02
    breq envia_um
    cpi config_state, 0x03
    breq envia_dm

envia_us:
    ; Envia " Ajustando a unidade dos segundos"
    ldi r24, low(us_string)
    ldi r25, high(us_string)
    call send_string
    ret

envia_ds:
    ; Envia " Ajustando a dezena dos segundos"
    ldi r24, low(ds_string)
    ldi r25, high(ds_string)
    call send_string
    ret

envia_um:
    ; Envia " Ajustando a unidade dos minutos"
    ldi r24, low(um_string)
    ldi r25, high(um_string)
    call send_string
    ret

envia_dm:
    ; Envia " Ajustando a dezena dos minutos"
    ldi r24, low(dm_string)
    ldi r25, high(dm_string)
    call send_string

    ret


; Gera um delay de 50ms (para debouncing)
delay_50ms:
    .equ clk_khz = clk / 1000 ; 16000
    .equ c50ms = (100*clk_khz)/2 - 1 ; Constante para 50 ms
	ldi r25,HIGH(c50ms)
	ldi r24,LOW(c50ms)
delay:
    sbiw R24,1 ; Contagem regressiva
	brne delay ; até zero
    ret


; Rotina: poll_mode
; Descrição: Verifica se o botão MODE está pressionado e muda o estado do sistema
; Parâmetros: state, estado atual do sistema
poll_mode:
    sbis PINB, pin_mode      ; Pula se o botão MODE estiver pressionado
    rjmp end_poll ; Se o botão não estiver pressionado, encerra

    ; Debouncing
    call delay_50ms         ; Espera 50ms para debouncing

    sbis PINB, pin_mode      ; Verifica novamente após o delay
    rjmp mode_pressed      ; Se ainda estiver pressionado, considera como um pressionamento válido

mode_not_pressed:
    rjmp end_poll

mode_pressed:
    inc state ; Passa para o próximo estado
    cpi state, 1
    breq cronometro_start ; Se o estado for 1, inicia o cronômetro
    cpi state, 2
    ; Setup para o modo de configuração
    brne not_config
    mov config_minutes_counter, minutes_counter
    mov config_secs_counter, secs_counter
    call atualiza_display
    clr config_state

; Caso o estado seja diferente de 1 ou 2, volta para o estado 0
not_config:
    cpi state, 3
    brne end_poll
    clr state
    rjmp end_poll

; Lógica para iniciar o cronômetro
cronometro_start:
    call uart_modo_cronometro_zero
    rjmp end_poll

end_poll:
    ret

; Rotina: poll_start
; Descrição: Verifica se o botão START está pressionado e executa a lógica de acordo com o estado atual
; Parâmetros: state, estado atual do sistema
poll_start:
    sbis PINB, pin_start        ; Pula se o botão START estiver pressionado
    rjmp end_poll               ; Se o botão não estiver pressionado, encerra

    ; Debouncing
    call delay_50ms             ; Espera 50ms para debouncing

    sbis PINB, pin_start        ; Verifica novamente após o delay
    rjmp start_pressed         ; Se ainda estiver pressionado, considera como um pressionamento válido

start_not_pressed:
    rjmp end_poll

start_pressed:
    ; Compara temp com 0x01 (modo cronômetro)
    cpi state, 0x01
    call inicia_para_tempo

    ; Compara temp com 0x02 (modo configuração)
    cpi state, 0x02
    call ajusta_tempo

    rjmp end_poll

; Rotina: poll_reset
; Descrição: Verifica se o botão RESET está pressionado e executa a lógica de acordo com o estado atual
; Parâmetros: state, estado atual do sistema
poll_reset:
    sbis PINB, pin_reset        ; Pula se o botão RESET estiver pressionado
    rjmp end_poll               ; Se o botão não estiver pressionado, encerra

    ; Debouncing
    call delay_50ms             ; Espera 50ms para debouncing

    sbis PINB, pin_reset        ; Verifica novamente após o delay
    rjmp reset_pressed         ; Se ainda estiver pressionado, considera como um pressionamento válido

reset_not_pressed:
    rjmp end_poll

reset_pressed:
    ; Compara temp com 0x01 (modo cronômetro)
    cpi state, 0x01
    call reinicia_tempo

    ; Compara temp com 0x02 (modo configuração)
    cpi state, 0x02
    call aplica_ajuste

    rjmp end_poll


; Rotina para enviar uma string armazenada na memória do programa via serial
; Entrada: ponteiro para a string
; Saída: nenhuma
send_string:
    lpm temp, Z+               ; Carrega um byte da string na memória do programa para temp
    tst temp                   ; Testa se o caractere é 0 (fim da string)
    breq end_string           ; Se for 0, termina a rotina
    call send_char            ; Envia o caractere
    rjmp send_string           ; Vai para o próximo caractere
end_string:
    ret

; Rotina para enviar um caractere via serial
; Entrada: temp, caractere a ser enviado
; Saída: nenhuma
send_char:
    push temp_seg
    lds temp_seg, UCSR0A            ; Carrega o status da UART
    sbrs temp_seg, UDRE0            ; Espera o registro de dados estar pronto para receber o próximo byte
    rjmp send_char                  ; Loop até estar pronto
    sts UDR0, temp                  ; Envia o caractere
    pop temp_seg
	ret

; Rotina auxiliar para transformar os valores de minutos e segundos em strings
; Entrada: secs_counter, minutes_counter
; Saída: temp_seg, temp (valores ASCII dos minutos e segundos, respectivamente)
to_ascii:
    ldi temp, 0x30
    add temp, secs_counter
    ldi temp_seg, 0x30
    add temp_seg, minutes_counter
    ret


; Sub-rotina para exibir dois dígitos
DISPLAY_TWO_DIGITS:
    push temp

    ; Exibe o dígito das dezenas
    lsr temp
    lsr temp
    andi temp, 0x0F
    call DISPLAY_DIGIT

    ; Exibe o dígito das unidades
    pop temp
    andi temp, 0x0F
    call DISPLAY_DIGIT

    ret

; Sub-rotina para exibir um dígito
DISPLAY_DIGIT:
    mov r16, temp
	lds temp, SEG_TABLE_START
    add r16, temp
    out SEGMENT_PORT, r16
    out DIGIT_PORT, r16
    ret

; Interrupção do Timer1 (1 segundo)
; A cada segundo, incrementa o contador de segundos e atualiza o display
; Dependendo do estado atual, pode atualizar o tempo no modo de cronômetro ou configuração
TIMER1_OVF_ISR:
    push temp
    in temp, SREG
    push temp

    mov temp, state

    ; Compara temp com 0x00 (modo relógio)
    cpi temp, 0x00
    breq atualiza_modo_relogio

    ; Compara temp com 0x01 (modo cronômetro)
    cpi temp, 0x01
    breq atualiza_modo_cronometro

    ; Compara temp com 0x02 (modo cronômetro)
    cpi temp, 0x02
    breq atualiza_modo_configuracao

    rjmp END_ISR

atualiza_modo_relogio:
    inc secs_counter
    cpi secs_counter, 60
    brne no_overflow

    ldi secs_counter, 0x0
    inc minutes_counter
    cpi minutes_counter, 60
    brne no_overflow

    ldi minutes_counter, 0X0

no_overflow:
    call atualiza_display
    call uart_modo_relogio
    rjmp END_ISR

atualiza_modo_cronometro:
    cpi flag_start, 1; Se a flag não estiver 1, pular incremento de tempo
    brne END_ISR

    inc temp_secs_counter
    cpi temp_secs_counter, 60
    brne END_ISR

    ldi temp_secs_counter, 0x0
    inc temp_minutes_counter
    cpi temp_minutes_counter, 60
    brne END_ISR

    ldi temp_minutes_counter, 0X0

    call atualiza_display

    rjmp END_ISR

; Lógica para atualizar o tempo no modo de configuração de acordo com o estado atual de config_state
atualiza_modo_configuracao:
    cpi config_state, 0x00
    breq atualiza_seg_un

    cpi config_state, 0x01
    breq atualiza_seg_dez

    cpi config_state, 0x02
    breq atualiza_min_un

    cpi config_state, 0x02
    breq atualiza_min_dez

    rjmp END_ISR

atualiza_seg_un:
    ; Incrementa config_secs_counter em 1
    mov temp, config_secs_counter ; Carrega config_secs_counter em temp
    inc temp ; Incrementa temp
    andi temp, 0x0F ; Obtém o último dígito de temp
    cpi temp, 0x0A ; Compara se o último dígito é 10
    brne skip_subtraction_seg_un
    subi temp, 10 ; Subtrai 10 se o último dígito é 10
skip_subtraction_seg_un:
    mov config_secs_counter, temp ; Armazena o valor atualizado de volta em config_secs_counter
    rjmp END_ISR ;

atualiza_seg_dez:
    ; Incrementa config_secs_counter em 10
    mov temp, config_secs_counter
    subi temp, 246 ; 256 - 10 = 246, para simular adição de 10 com underflow
    cpi temp, 60
    brlo no_decrement_seg_dez
    subi temp, 196 ; 256 - 60 = 196, para resetar ao início caso >= 60
no_decrement_seg_dez:
    mov config_secs_counter, temp
    rjmp END_ISR

atualiza_min_un:
    ; Incrementa config_minutes_counter em 1
    mov r16, config_minutes_counter
    inc r16
    andi r16, 0x0F
    cpi r16, 0x0A
    brne skip_subtraction_min_un
    subi r16, 10
skip_subtraction_min_un:
    mov config_minutes_counter, r16
    rjmp END_ISR

atualiza_min_dez:
    ; Incrementa config_minutes_counter em 10
    mov r16, config_minutes_counter
    subi r16, 246 ; Mesma lógica de adição com underflow
    cpi r16, 60
    brlo no_decrement_min_dez
    subi r16, 196 ; Reseta se o valor for >= 60
no_decrement_min_dez:
    mov config_minutes_counter, r16
    rjmp END_ISR

END_ISR:
    ; Fim da interrupçâo, restauração do contexto de SREG
    pop temp
    out SREG, temp
    pop temp
    reti