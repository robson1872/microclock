jmp reset
.org OC1Aaddr
jmp TIMER1_OVF_ISR

reset:

; Pinos
.equ pin_mode = PINB0    ; Pino para o botão MODE
.equ pin_start = PINB1   ; Pino para o botão START
.equ pin_reset = PINB2   ; Pino para o botão RESET
.equ pin_led = PORTC0          ; Pino para o led

; Definições de variáveis
.def TEMP_SECS_COUNTER = r16   ; Registrador para armazenar os segundos (cronômetro)
.def TEMP_MINUTES_COUNTER = r17   ; Registrador para armazenar os minutos (cronômetro)
.def state = r18          ; Registrador para armazenar o state atual
.def FLAG_START = r19      ; Registrador para a flag do botão START
.def FLAG_RESET = r20      ; Registrador para a flag do botão RESET
.def FLAG_MODE = r21       ; Registrador para a flag do botão MODE
.def MINUTES_COUNTER = r24          ; Registrador para armazenar o minuto
.def SECS_COUNTER = r25         ; Registrador para armazenar o segundo
.def temp = r23            ; Registrador temporário 


; Configuração de interrupções
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

;Inicialização da stack
ldi temp, low(RAMEND)
out SPL, temp
ldi temp, high(RAMEND)
out SPH, temp

; Configuração de portas
ldi temp, (1<<pin_mode)|(1<<pin_start)|(1<<pin_reset)  ; Configura pinos de botão como entrada
out DDRB, temp          ; Define porta B como saída
ldi temp, (1<<pin_led)      ; Configura pino do pin_led como saída
out DDRC, temp          ; Define porta C como saída

; Configuração inicial dos registradores
clr TEMP_SECS_COUNTER
clr TEMP_MINUTES_COUNTER
clr SECS_COUNTER
clr MINUTES_COUNTER
clr state

loop:
    ;limpa flags imediatamente após trocar de modo
    clr FLAG_START
    clr FLAG_RESET

    lds temp, state ; Carrega o estado atual de funcionamento em temp

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
    ; Verifica botão MODE
    call poll_mode

    ; Verifica botão START
    call poll_start

    ; Verifica botão RESET
    call poll_reset

    rjmp loop

; Lógica para iniciar/parar o cronômetro
inicia_para_tempo:
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

fim_reinicia_tempo:
    ret

; TODO: Lógica para andar pelo tempo no modo de configuração
ajusta_tempo:
    ret

; TODO: Lógica para aplicar o ajuste de tempo
aplica_ajuste:
    ret

; TODO: Lógica para mostrar minutos e segundos no display de 7 segmentos
atualiza_display:
    ret

; TODO: Lógica para imprimir na serial (Modo 1)
uart_modo_relogio:
    ret 

; TODO: Lógica para imprimir na serial (Modo 2)
uart_modo_cronometro:
    ret 

; TODO: Lógica para imprimir na serial (Modo 3)
uart_modo_configuracao:
    ret 

; TODO: Rotina de interrupção do Timer0 (0.5 segundo)
TIMER0_OVF_ISR:
    reti

; Interrupção do Timer1 (1 segundo)
TIMER1_OVF_ISR:
    push temp
    in temp, SREG
    push temp

    lds temp, state

    ; Compara temp com 0x00 (modo relógio)
    cpi temp, 0x00
    breq atualiza_modo_relogio

    ; Compara temp com 0x01 (modo cronômetro)
    cpi temp, 0x01
    breq atualiza_modo_cronometro

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
    call uart_modo_cronometro

    rjmp END_ISR

END_ISR:
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
    cpi state, 3
    brne end_poll
    clr state

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
