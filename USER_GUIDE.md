# 使用手册
## C板接口定义
### SWD: 
DIO CLK GND 3.3V
### UART2(USART1):
RXD TXD GND 5V
### UART1(USART6):
GND TXD RXD
### CAN1:
CANL CANH
### CAN2:
5V GND CANH CANL
### KEY:
PA0 (PULL_UP)
### RGB LED:
| TIM5 | CH1  | CH2  | CH3  |
|------|------|------|------|
| LED  | B    | G    | R    |
| GPIO | PH10 | PH11 | PH12 |
### BUZZER:
PD14 (TIM4_CH3)
### 5V:
PC8 (TIM3_CH3)
### BOOT:
| BOOT0 | 3.3V |
|-------|------|
| BOOT1 | 3.3V |
