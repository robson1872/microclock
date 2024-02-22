; Pinos
.equ pin_mode = PINB0    ; Pino para o botão MODE
.equ pin_start = PINB1   ; Pino para o botão START
.equ pin_reset = PINB2   ; Pino para o botão RESET
.equ pin_led = PORTC0          ; Pino para o led

; Definições de variáveis
.def reg_temporizador1 = r16   ; Registrador para temporização 1
.def reg_temporizador2 = r17   ; Registrador para temporização 2
.def state = r18          ; Registrador para armazenar o state atual
.def FLAG_START = r19      ; Registrador para a flag do botão START
.def FLAG_RESET = r20      ; Registrador para a flag do botão RESET
.def FLAG_MODE = r21       ; Registrador para a flag do botão MODE
.def CONT_TEMP_CRON = r22  ; Registrador para contar tempo do cronômetro
.def HORA = r23            ; Registrador para armazenar a hora
.def MINUTO = r24          ; Registrador para armazenar o minuto
.def SEGUNDO = r25         ; Registrador para armazenar o segundo

;TODO: Configuração de registradores

; Configuração de portas
ldi r16, (1<<pin_mode)|(1<<pin_start)|(1<<pin_reset)  ; Configura pinos de botão como entrada
out DDRB, r16          ; Define porta B como saída
ldi r16, (1<<pin_led)      ; Configura pino do pin_led como saída
out DDRC, r16          ; Define porta C como saída

; TODO: Configuração de interrupções

; Main Looop
loop:
    ; VerificaBotão MODE
    sbic PINB, pin_mode
    rjmp modo_de_operacao

    ; Verifica botão START
    sbic PINB, pin_start
    rjmp inicia_para_tempo

    ; Verifica botão RESET
    sbic PINB, pin_reset
    rjmp reinicia_tempo

    ; Lógica de operação baseada no state atual
    cpi state, 1
    brne modo_cronometro
    ; MODO 1: Apresentação do tempo normal
    rjmp loop

;TODO:  Lógica para o modo do cronômetro
modo_cronometro:
    rjmp loop

;TODO: Lógica para alternar entre os modos de operação
modo_de_operacao:
    rjmp loop


;TODO: Lógica para iniciar/parar o cronômetro ou navegar pelo ajuste de tempo
inicia_para_tempo:
    rjmp loop

;TODO: Lógica para zerar o cronômetro ou aplicar o ajuste de tempo
reinicia_tempo:

    rjmp loop

;  Rotina de interrupção do Timer0 (0.5 segundo)
TIMER0_OVF_ISR:
    reti

; TODO: interrupção do Timer1 (1 segundo)
TIMER1_OVF_ISR:
    reti
