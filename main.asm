; Pinos
.equ pin_mode = PINB0    ; Pino para o bot�o MODE
.equ pin_start = PINB1   ; Pino para o bot�o START
.equ pin_reset = PINB2   ; Pino para o bot�o RESET
.equ pin_led = PORTC0          ; Pino para o led

; Defini��es de vari�veis
.def reg_temporizador1 = r16   ; Registrador para temporiza��o 1
.def reg_temporizador2 = r17   ; Registrador para temporiza��o 2
.def state = r18          ; Registrador para armazenar o state atual
.def FLAG_START = r19      ; Registrador para a flag do bot�o START
.def FLAG_RESET = r20      ; Registrador para a flag do bot�o RESET
.def FLAG_MODE = r21       ; Registrador para a flag do bot�o MODE
.def CONT_TEMP_CRON = r22  ; Registrador para contar tempo do cron�metro
.def HORA = r23            ; Registrador para armazenar a hora
.def MINUTO = r24          ; Registrador para armazenar o minuto
.def SEGUNDO = r25         ; Registrador para armazenar o segundo

;TODO: Configura��o de registradores

; Configura��o de portas
ldi r16, (1<<pin_mode)|(1<<pin_start)|(1<<pin_reset)  ; Configura pinos de bot�o como entrada
out DDRB, r16          ; Define porta B como sa�da
ldi r16, (1<<pin_led)      ; Configura pino do pin_led como sa�da
out DDRC, r16          ; Define porta C como sa�da

; TODO: Configura��o de interrup��es

; Main Looop
loop:
    ; VerificaBot�o MODE
    sbic PINB, pin_mode
    rjmp modo_de_operacao

    ; Verifica bot�o START
    sbic PINB, pin_start
    rjmp inicia_para_tempo

    ; Verifica bot�o RESET
    sbic PINB, pin_reset
    rjmp reinicia_tempo

    ; L�gica de opera��o baseada no state atual
    cpi state, 1
    brne modo_cronometro
    ; MODO 1: Apresenta��o do tempo normal
    rjmp loop

;TODO:  L�gica para o modo do cron�metro
modo_cronometro:
    rjmp loop

;TODO: L�gica para alternar entre os modos de opera��o
modo_de_operacao:
    rjmp loop


;TODO: L�gica para iniciar/parar o cron�metro ou navegar pelo ajuste de tempo
inicia_para_tempo:
    rjmp loop

;TODO: L�gica para zerar o cron�metro ou aplicar o ajuste de tempo
reinicia_tempo:

    rjmp loop

;  Rotina de interrup��o do Timer0 (0.5 segundo)
TIMER0_OVF_ISR:
    reti

; TODO: interrup��o do Timer1 (1 segundo)
TIMER1_OVF_ISR:
    reti
