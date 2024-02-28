; Projeto Relógio Digital
; Disciplina: Microcontroladores e Aplicações
; Professor: Dr. Erick de Andrade Barboza
; Alunos:
;  - Robson Bruno Alves Pinheiro
;  - Victor Alexandre da R. Monteiro Miranda

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
; TODO: Padronizar o casing das variáveis
.def TEMP_SECS_COUNTER = r16   ; Registrador para armazenar os segundos (cronômetro)
.def TEMP_MINUTES_COUNTER = r17   ; Registrador para armazenar os minutos (cronômetro)
.def state = r18          ; Registrador para armazenar o state atual
.def FLAG_START = r19      ; Registrador para a flag do botão START
.def FLAG_RESET = r20      ; Registrador para a flag do botão RESET
.def FLAG_MODE = r21       ; Registrador para a flag do botão MODE
.def MINUTES_COUNTER = r22          ; Registrador para armazenar o minuto
.def SECS_COUNTER = r23         ; Registrador para armazenar o segundo
.def string_pointer = r24       ; Registrador para armazenar o ponteiro da string
.def string_pointer2 = r25      ; Registrador para armazenar o ponteiro da string
.def config_state = r26           ; Registrador para armazenar o estado atual do modo de configuração
.def CONFIG_MINUTES_COUNTER = r27 ; Registrador para armazenar os segundos (configuração)
.def CONFIG_SECS_COUNTER = r28  ; Registrador para armazenar os minutos (configuração)
.def temp = r24            ; Registrador temporário
.def temp_seg = r25            ; Registrador temporário 2

; Configuração UART
; 16Mhz, 9600 baud, UBRR = 103
.equ UBRRvalue = 103
ldi temp, high (UBRRvalue) ;baud rate
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

; configuração de interrupções
#define DELAY 1 ;segundos
.equ clk 16.0e6 ;clock speed
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
ldi temp, (1<<pin_mode)|(1<<pin_start)|(1<<pin_reset)  ; Configura pinos de botão como entrada
out DDRB, temp          ; Define porta B como saída
ldi temp, (1<<pin_led)      ; Configura pino do pin_led como saída
out DDRC, temp          ; Define porta C como saída

; configuração inicial dos registradores
clr TEMP_SECS_COUNTER
clr TEMP_MINUTES_COUNTER
clr SECS_COUNTER
clr MINUTES_COUNTER
clr state ; Inicializa o estado do sistema como 0 (modo relógio)

; Loop principal do programa
loop:
    ;limpa flags imediatamente após trocar de modo
    clr FLAG_START
    clr FLAG_RESET

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
modo_relogio:
    call atualiza_display

    ; Verifica botão MODE
    call poll_mode

    rjmp loop

;(MODO 2 DE OPERAÇÃO)
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
modo_configuracao:
    call atualiza_display

    ; Verifica botão MODE
    call poll_mode

    ; Verifica botão START
    call poll_start

    ; Verifica botão RESET
    call poll_reset

    rjmp loop

; Lógica para iniciar/parar o cronômetro
inicia_para_tempo:
    call uart_modo_cronometro_start
    push temp
    ldi temp, 0x01
    eor FLAG_START, temp
    pop temp
    ret

; Lógica para zerar o cronômetro
reinicia_tempo:
    cpi FLAG_START, 0; Se a flag não estiver 1, pular rotina
    brne fim_reinicia_tempo

    clr TEMP_MINUTES_COUNTER
    clr TEMP_SECS_COUNTER

    call atualiza_display
    call uart_modo_cronometro_reset

fim_reinicia_tempo:
    ret

; Lógica para andar pelo tempo no modo de configuração
; Alterna entre os possíveis modos: Ajustes de unidade dos segundos, dezena dos segundos, unidade dos minutos e dezena dos minutos
ajusta_tempo:
    inc config_state
    cpi config_state, 0x03
    brne fim_ajusta_tempo
    clr config_state

fim_ajusta_tempo:
    ret

; Lógica para aplicar o ajuste de tempo
aplica_ajuste:
    mov MINUTES_COUNTER, CONFIG_MINUTES_COUNTER
    mov SECS_COUNTER, CONFIG_SECS_COUNTER
    ret

; Lógica para mostrar minutos e segundos no display de 7 segmentos
atualiza_display:
    lds temp, seconds
    call DISPLAY_TWO_DIGITS
    lds temp, minutes
    call DISPLAY_TWO_DIGITS
    ret

; Lógica para imprimir na serial (Modo 1)
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

; Lógica para imprimir na serial (Modo 2)
uart_modo_cronometro:
    ; Envia "[MODO 2]"
    ldi r24, low(mode2_string)
    ldi r25, high(mode2_string)
    call send_string

    ret

; Lógica para imprimir na serial (Modo 2)
uart_modo_cronometro_zero:
    call uart_modo_cronometro ; Envia "[MODO 2]"

    ; Envia " ZERO"
    ldi r24, low(zero_string)
    ldi r25, high(zero_string)
    call send_string

    ret

; Lógica para imprimir na serial (Modo 2)
uart_modo_cronometro_start:
    call uart_modo_cronometro ; Envia "[MODO 2]"

    ; Envia " START"
    ldi r24, low(start_string)
    ldi r25, high(start_string)
    call send_string

    ret

; Lógica para imprimir na serial (Modo 2)
uart_modo_cronometro_reset:
    call uart_modo_cronometro ; Envia "[MODO 2]"

    ; Envia " RESET"
    ldi r24, low(reset_string)
    ldi r25, high(reset_string)
    call send_string

    ret

; TODO: Lógica para imprimir na serial (Modo 3)
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

; Rotina de interrupção do Timer0 (0.5 segundo)
TIMER0_OVF_ISR:
    cpi state, 0x2
    brne END_ISR_1

    ;Lógica para piscar o display 7 segmentos de acordo com o estado atual de config_state

END_ISR_1:
    reti

; interrupção do Timer1 (1 segundo)
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
    inc SECS_COUNTER
    cpi SECS_COUNTER, 60
    brne no_overflow

    ldi SECS_COUNTER, 0x0
    inc MINUTES_COUNTER
    cpi MINUTES_COUNTER, 60
    brne no_overflow

    ldi MINUTES_COUNTER, 0X0

no_overflow:
    call atualiza_display
    call uart_modo_relogio
    rjmp END_ISR

atualiza_modo_cronometro:
    cpi FLAG_START, 1; Se a flag não estiver 1, pular incremento de tempo
    brne END_ISR

    inc TEMP_SECS_COUNTER
    cpi TEMP_SECS_COUNTER, 60
    brne END_ISR

    ldi TEMP_SECS_COUNTER, 0x0
    inc TEMP_MINUTES_COUNTER
    cpi TEMP_MINUTES_COUNTER, 60
    brne END_ISR

    ldi TEMP_MINUTES_COUNTER, 0X0

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
    ; Incrementa CONFIG_SECS_COUNTER em 1
    lds temp, CONFIG_SECS_COUNTER ; Carrega CONFIG_SECS_COUNTER em temp
    inc temp ; Incrementa temp
    andi temp, 0x0F ; Obtém o último dígito de temp
    cpi temp, 0x0A ; Compara se o último dígito é 10
    brne skip_subtraction_seg_un
    subi temp, 10 ; Subtrai 10 se o último dígito é 10
skip_subtraction_seg_un:
    sts CONFIG_SECS_COUNTER, temp ; Armazena o valor atualizado de volta em CONFIG_SECS_COUNTER
    rjmp END_ISR ;

atualiza_seg_dez:
    ; Incrementa CONFIG_SECS_COUNTER em 10
    lds temp, CONFIG_SECS_COUNTER
    subi temp, 246 ; 256 - 10 = 246, para simular adição de 10 com underflow
    cpi temp, 60
    brlo no_decrement_seg_dez
    subi temp, 196 ; 256 - 60 = 196, para resetar ao início caso >= 60
no_decrement_seg_dez:
    sts CONFIG_SECS_COUNTER, temp
    rjmp END_ISR

atualiza_min_un:
    ; Incrementa CONFIG_MINUTES_COUNTER em 1
    lds r16, CONFIG_MINUTES_COUNTER
    inc r16
    andi r16, 0x0F
    cpi r16, 0x0A
    brne skip_subtraction_min_un
    subi r16, 10
skip_subtraction_min_un:
    sts CONFIG_MINUTES_COUNTER, r16
    rjmp END_ISR

atualiza_min_dez:
    ; Incrementa CONFIG_MINUTES_COUNTER em 10
    lds r16, CONFIG_MINUTES_COUNTER
    subi r16, 246 ; Mesma lógica de adição com underflow
    cpi r16, 60
    brlo no_decrement_min_dez
    subi r16, 196 ; Reseta se o valor for >= 60
no_decrement_min_dez:
    sts CONFIG_MINUTES_COUNTER, r16
    rjmp END_ISR

END_ISR:
    ; Fim da interrupçâo, restauração do contexto de SREG
    pop temp
    out SREG, temp
    pop temp
    reti

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


; Verificar botão (pin_mode)
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
    inc state
    cpi state, 1
    breq cronometro_start
    cpi state, 2
    ; Setup para o modo de configuração
    brne not_config
    mov CONFIG_MINUTES_COUNTER, MINUTES_COUNTER
    mov CONFIG_SECS_COUNTER, SECS_COUNTER
    call atualiza_display
    clr config_state

not_config:
    cpi state, 3
    brne end_poll
    clr state
    rjmp end_poll

cronometro_start:
    call uart_modo_cronometro_zero
    rjmp

end_poll:
    ret

; Verificar botão (pin_start)
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

; Verificar botão (pin_reset)
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
    push string_pointer
    push string_pointer2
next_char:
    lpm temp, Z+               ; Carrega um byte da string na memória do programa para temp
    tst temp                   ; Testa se o caractere é 0 (fim da string)
    breq end_string           ; Se for 0, termina a rotina
    call send_char            ; Envia o caractere
    rjmp next_char            ; Vai para o próximo caractere
end_string:
    pop string_pointer_2
    pop string_pointer
    ret

; Rotina para enviar um caractere via serial
; Entrada: temp, caractere a ser enviado
; Saída: nenhuma
send_char:
    push
    lds temp_seg, UCSR0A            ; Carrega o status da UART
    sbrs temp_seg, UDRE0            ; Espera o registro de dados estar pronto para receber o próximo byte
    rjmp send_char                  ; Loop até estar pronto
    sts UDR0, temp                  ; Envia o caractere
    ret

; Rotina auxiliar para transformar os valores de minutos e segundos em strings
; Entrada: SECS_COUNTER, MINUTES_COUNTER
; Saída: temp_seg, temp (valores ASCII dos minutos e segundos, respectivamente)
to_ascii:
    ldi temp, 0x30
    add temp, SECS_COUNTER
    ldi temp_seg, 0x30
    add temp_seg, MINUTES_COUNTER
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
    add r16, SEG_TABLE_START
    out SEGMENT_PORT, r16
    out DIGIT_PORT, r16
    ret
