jmp reset
.org OC1Aaddr
jmp TIMER1_OVF_ISR

reset:

; Pinos
.equ pin_mode = PINB0    ; Pino para o bot�o MODE
.equ pin_start = PINB1   ; Pino para o bot�o START
.equ pin_reset = PINB2   ; Pino para o bot�o RESET
.equ pin_led = PORTC0          ; Pino para o led

; Defini��es de vari�veis
.def TEMP_SECS_COUNTER = r16   ; Registrador para armazenar os segundos (cron�metro)
.def TEMP_MINUTES_COUNTER = r17   ; Registrador para armazenar os minutos (cron�metro)
.def state = r18          ; Registrador para armazenar o state atual
.def FLAG_START = r19      ; Registrador para a flag do bot�o START
.def FLAG_RESET = r20      ; Registrador para a flag do bot�o RESET
.def FLAG_MODE = r21       ; Registrador para a flag do bot�o MODE
.def MINUTES_COUNTER = r24          ; Registrador para armazenar o minuto
.def SECS_COUNTER = r25         ; Registrador para armazenar o segundo
.def temp = r23            ; Registrador tempor�rio 


; Configura��o de interrup��es
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
; Fim da configura��o de interrup��es

;Inicializa��o da stack
ldi temp, low(RAMEND)
out SPL, temp
ldi temp, high(RAMEND)
out SPH, temp

; Configura��o de portas
ldi temp, (1<<pin_mode)|(1<<pin_start)|(1<<pin_reset)  ; Configura pinos de bot�o como entrada
out DDRB, temp          ; Define porta B como sa�da
ldi temp, (1<<pin_led)      ; Configura pino do pin_led como sa�da
out DDRC, temp          ; Define porta C como sa�da

; Configura��o inicial dos registradores
clr TEMP_SECS_COUNTER
clr TEMP_MINUTES_COUNTER
clr SECS_COUNTER
clr MINUTES_COUNTER
clr state

loop:
    ;limpa flags imediatamente ap�s trocar de modo
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
    

;(MODO 1 DE OPERA��O)
modo_relogio:
    call atualiza_display

    ; Verifica bot�o MODE
    call poll_mode

    rjmp loop

;(MODO 2 DE OPERA��O)
modo_cronometro:
    call atualiza_display

    ; Verifica bot�o MODE
    call poll_mode

    ; Verifica bot�o START
    call poll_start

    ; Verifica bot�o RESET
    call poll_reset

    rjmp loop

;(MODO 3 DE OPERA��O)
modo_configuracao:
    ; Verifica bot�o MODE
    call poll_mode

    ; Verifica bot�o START
    call poll_start

    ; Verifica bot�o RESET
    call poll_reset

    rjmp loop

; L�gica para iniciar/parar o cron�metro
inicia_para_tempo:
    push temp
    ldi temp, 0x01
    eor FLAG_START, temp
    pop temp
    ret

; L�gica para zerar o cron�metro 
reinicia_tempo:
    cpi FLAG_START, 0; Se a flag n�o estiver 1, pular rotina
    brne fim_reinicia_tempo

    clr TEMP_MINUTES_COUNTER
    clr TEMP_SECS_COUNTER

    call atualiza_display

fim_reinicia_tempo:
    ret

; TODO: L�gica para andar pelo tempo no modo de configura��o
ajusta_tempo:
    ret

; TODO: L�gica para aplicar o ajuste de tempo
aplica_ajuste:
    ret

; TODO: L�gica para mostrar minutos e segundos no display de 7 segmentos
atualiza_display:
    ret

; TODO: L�gica para imprimir na serial (Modo 1)
uart_modo_relogio:
    ret 

; TODO: L�gica para imprimir na serial (Modo 2)
uart_modo_cronometro:
    ret 

; TODO: L�gica para imprimir na serial (Modo 3)
uart_modo_configuracao:
    ret 

; TODO: Rotina de interrup��o do Timer0 (0.5 segundo)
TIMER0_OVF_ISR:
    reti

; Interrup��o do Timer1 (1 segundo)
TIMER1_OVF_ISR:
    push temp
    in temp, SREG
    push temp

    lds temp, state

    ; Compara temp com 0x00 (modo rel�gio)
    cpi temp, 0x00
    breq atualiza_modo_relogio

    ; Compara temp com 0x01 (modo cron�metro)
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
    cpi FLAG_START, 1; Se a flag n�o estiver 1, pular incremento de tempo
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
	brne delay ; at� zero 
    ret


; Verificar bot�o (pin_mode)
poll_mode:
    sbis PINB, pin_mode      ; Pula se o bot�o MODE estiver pressionado
    rjmp end_poll ; Se o bot�o n�o estiver pressionado, encerra

    ; Debouncing
    call delay_50ms         ; Espera 50ms para debouncing

    sbis PINB, pin_mode      ; Verifica novamente ap�s o delay
    rjmp mode_pressed      ; Se ainda estiver pressionado, considera como um pressionamento v�lido

mode_not_pressed:
    rjmp end_poll

mode_pressed:
    inc state
    cpi state, 3
    brne end_poll
    clr state

end_poll:
    ret

; Verificar bot�o (pin_start)
poll_start:
    sbis PINB, pin_start        ; Pula se o bot�o START estiver pressionado
    rjmp end_poll               ; Se o bot�o n�o estiver pressionado, encerra

    ; Debouncing
    call delay_50ms             ; Espera 50ms para debouncing

    sbis PINB, pin_start        ; Verifica novamente ap�s o delay
    rjmp start_pressed         ; Se ainda estiver pressionado, considera como um pressionamento v�lido

start_not_pressed:
    rjmp end_poll

start_pressed:
    ; Compara temp com 0x01 (modo cron�metro)
    cpi state, 0x01
    call inicia_para_tempo

    ; Compara temp com 0x02 (modo configura��o)
    cpi state, 0x02
    call ajusta_tempo

    rjmp end_poll

; Verificar bot�o (pin_reset)
poll_reset:
    sbis PINB, pin_reset        ; Pula se o bot�o RESET estiver pressionado
    rjmp end_poll               ; Se o bot�o n�o estiver pressionado, encerra

    ; Debouncing
    call delay_50ms             ; Espera 50ms para debouncing

    sbis PINB, pin_reset        ; Verifica novamente ap�s o delay
    rjmp reset_pressed         ; Se ainda estiver pressionado, considera como um pressionamento v�lido

reset_not_pressed:
    rjmp end_poll

reset_pressed:
    ; Compara temp com 0x01 (modo cron�metro)
    cpi state, 0x01
    call reinicia_tempo

    ; Compara temp com 0x02 (modo configura��o)
    cpi state, 0x02
    call aplica_ajuste

    rjmp end_poll
