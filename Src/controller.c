//
// Created by Luke Nel on 17/05/2023.
//

#define BUZZER_ENABLE

#include "controller.h"
#include "main.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "lcd.h"
#include "eeprom.h"
#include "oled.h"

static const char edit_pid_pre[] = "X ";
static const char edit_pid_suf[] = "";
#define PID_VAL_WIDTH       8
#define PID_VAL_PREC        4
#define PID_EDIT_MIN        (strlen(edit_pid_pre)-1)
#define PID_EDIT_MID        (PID_EDIT_MIN+PID_VAL_WIDTH-PID_VAL_PREC)
#define PID_EDIT_MAX        (PID_EDIT_MIN+PID_VAL_WIDTH+1)

static const char edit_s_pre[] = "X 1.6V= ";
static const char edit_s_suf[] = " RPM";
#define S_VAL_WIDTH         4
#define S_VAL_PREC          0
#define S_EDIT_MIN          (strlen(edit_s_pre)-1)
#define S_EDIT_MID          20
#define S_EDIT_MAX          (S_EDIT_MIN+S_VAL_WIDTH+1)

static const char edit_i_pre[] = "X 1.6V= ";
static const char edit_i_suf[] = " RPM";
#define I_VAL_WIDTH         4
#define I_VAL_PREC          0
#define I_EDIT_MIN          (strlen(edit_i_pre)-1)
#define I_EDIT_MID          20
#define I_EDIT_MAX          (I_EDIT_MIN+I_VAL_WIDTH+1)

static const char edit_o_pre[] = "X 50% = ";
static const char edit_o_suf[] = " V";
#define O_VAL_WIDTH         4
#define O_VAL_PREC          0
#define O_EDIT_MIN          (strlen(edit_o_pre)-1)
#define O_EDIT_MID          20
#define O_EDIT_MAX          (O_EDIT_MIN+O_VAL_WIDTH+1)

#define EEPROM_PARAMETERS_ADDR  0x00A0

#define ENCODER_RESET()     (TIM2->CNT = 0)
#define ENCODER_VAL         (TIM2->CNT>>2)
#define ENCODER_MAX(a)      (TIM2->ARR=(a)<<2)

#define ENC_COOLDOWN_START()    HAL_TIM_Base_Start_IT(&htim5)
#define ENC_COOLDOWN_STOP()     HAL_TIM_Base_Stop_IT(&htim5)

#define BTN_COOLDOWN_START()    HAL_TIM_Base_Start_IT(&htim9)
#define BTN_COOLDOWN_STOP()     HAL_TIM_Base_Stop_IT(&htim9)

#define DUTY_MIN        (DUTY_50*0.05)
#define DUTY_MAX        (DUTY_50*1.95)

#define ADC_MIN         0.0f
#define ADC_MAX         3.2f
#define ADC_MID         ((ADC_MIN+ADC_MAX)/2) // 1.6V
#define ADC_CONST       1285 // For 1V
#define ADC_OFFSET      50
#define ADC_BUFF_LEN    2

#define MIN(a,b)        (a<b ? a : b)
#define MAX(a,b)        (a>b ? a : b)
#define CLAMP(c, a, b)  MIN(MAX(c, a), b)

#define OUTPUT_VAL      TIM1->CCR2
#define SAMPLE_RATE     30e3

#define SCREEN_BUFF_LEN   128

static struct {
    float Kp;
    float Ki;
    float Kd;
    float S_50;
    float I_50;
    float O_50;
} parameters;

enum lcd_state {
    LCD_X = 0,
    LCD_Kp,
    LCD_Ki,
    LCD_Kd,
    LCD_S,
    LCD_I,
    LCD_O,
    LCD_MAIN,
    LCD_EDIT
};

static struct {
    float S_V;
    float I_V;
    float O_DC;
    int e_ss;

    double k_p;
    double k_i;
    double k_d;
    double C;

    uint16_t S_RPM;
    uint16_t I_RPM;
    float O_V;

    volatile uint16_t adc_buff[ADC_BUFF_LEN];
    volatile bool adc_done;

    enum lcd_state current_lcd;
    uint16_t old_pos;
    uint16_t old_digit;

    volatile bool enc_cooldown;
    volatile bool btn_cooldown;
    bool manual_control;
    bool first_entry;
    bool edit_digit;

    uint16_t S_buff[128];
    uint16_t* S_ptr;

    uint16_t I_buff[128];
    uint16_t* I_ptr;
} state;

static void Start_Buzzer(void) {
    HAL_TIM_Base_Stop_IT(&htim10);
#ifdef BUZZER_ENABLE
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
#endif
    HAL_TIM_Base_Start_IT(&htim10);
}

static void Update_Values(void) {
    EEPROM_Save(EEPROM_PARAMETERS_ADDR, &parameters, sizeof(parameters));

    state.k_p = parameters.Kp;
    state.k_i = parameters.Ki / SAMPLE_RATE;
    state.k_d = parameters.Kd * SAMPLE_RATE;
    state.C = DUTY_50 / parameters.S_50;
}

void Controller_Init(void) {
    printf("\n\n--- PID Controllinator 2000 ---\r\n\n");

    if (EEPROM_Init(&hi2c1) == HAL_OK) {
        printf("EEPROM initialized");
    } else {
        printf("!!Error initializing EEPROM");
    }
    printf(" on I2C1, address 0x%2X.\r\n", EEPROM_I2C_ADDR);

    if (LCD_Init(&hi2c1) == HAL_OK) {
        printf("LCD initialized");
    } else {
        printf("!!Error initializing LCD");
    }
    printf(" on I2C1, address 0x%2X.\r\n", LCD_I2C_ADDR);

    if (OLED_Init(&hi2c2) == HAL_OK) {
        printf("OLED initialized");
    } else {
        printf("!!Error initializing OLED");
    }
    printf(" on I2C2, address 0x%2X.\r\n", OLED_I2C_ADDR);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) == HAL_OK) {
        printf("PWM output initialized");
    } else {
        printf("!!Error initializing");
    }
    printf(" on TIM1.\r\n");

    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) == HAL_OK) {
        printf("Rotary encoder initialized");
    } else {
        printf("!!Error initializing rotary encoder");
    }
    printf(" on TIM2.\r\n");

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *) state.adc_buff, ADC_BUFF_LEN)
        == HAL_OK) {
        printf("ADC initialized.\r\n");
    } else {
        printf("!!Error initializing ADC.\r\n");
    }

    ENC_COOLDOWN_START();
    BTN_COOLDOWN_START();
//    CONTROLLER_START();
    Start_Buzzer();

    state.current_lcd = LCD_MAIN;
    state.manual_control = true;
    state.first_entry = true;

    state.S_ptr = state.S_buff;
    state.I_ptr = state.I_buff;

//    memset(&parameters, 0, sizeof(parameters));
//    EEPROM_Save(EEPROM_PARAMETERS_ADDR, &parameters, sizeof(parameters));
//    HAL_Delay(100);

    EEPROM_Load(EEPROM_PARAMETERS_ADDR, &parameters, sizeof(parameters));

    Update_Values();

    printf("\nController Parameters:\r\n");
    printf("Kp: %0*.*f\r\n", PID_VAL_WIDTH, PID_VAL_PREC, parameters.Kp);
    printf("Ki: %0*.*f\r\n", PID_VAL_WIDTH, PID_VAL_PREC, parameters.Ki);
    printf("Kd: %0*.*f\r\n", PID_VAL_WIDTH, PID_VAL_PREC, parameters.Kd);
    printf("S_50: %f\r\n", parameters.S_50);
    printf("I_50: %f\r\n", parameters.I_50);
    printf("O_50: %f\r\n", parameters.O_50);
}

void Controller_Welcome(void) {

    HAL_GPIO_WritePin(ControlLED_GPIO_Port, ControlLED_Pin, GPIO_PIN_RESET);

    LCD_Begin_Payload();
    LCD_Clear_Display();
    LCD_Enable(true, false, false);
    LCD_Set_Cursor(0, 0);
    LCD_Print_s("PID");
    LCD_Set_Cursor(1, 1);
    LCD_Print_s("Controllinator");
    LCD_Set_Cursor(0, 9);
    LCD_Print_f(2000.00f);
    LCD_End_Payload();
    LCD_Send_Payload();

    OLED_GotoXY(20, 22);
    OLED_Puts("P.I.D.", &Font_16x26, 1);
    OLED_UpdateScreen();

    HAL_Delay(2000);

    LCD_Begin_Payload();
    LCD_Clear_Display();
    LCD_End_Payload();
    LCD_Send_Payload();

    OLED_Clear();

    HAL_GPIO_WritePin(ControlLED_GPIO_Port, ControlLED_Pin, GPIO_PIN_SET);
}

void Set_LCD_Main(void) {
    static uint16_t old_S = 0;
    static uint16_t old_e = 0;
    char setpoint[10];
    char error_out[10];

    if (state.first_entry == true || (old_S != state.S_RPM || old_e != state.e_ss)) {
        state.first_entry = false;
        old_S = state.S_RPM;
        old_e = state.e_ss;

        LCD_Begin_Payload();
        LCD_Enable(true, false, false);
        LCD_Set_Cursor(0, 0);
        LCD_Print_s("S: ");
        sprintf(setpoint, "%04d RPM", old_S);
        LCD_Print_s(setpoint);
        LCD_Set_Cursor(1,0);
        LCD_Print_s("e: ");
        sprintf(error_out, "%+07.2f%%", (float)state.e_ss / (float)state.S_RPM * 100);
        LCD_Print_s(error_out);
        LCD_End_Payload();
        LCD_Send_Payload();
    }
    osDelay(500);
}

void Set_LCD_Edit(void) {
    if (state.first_entry == true || state.old_pos != ENCODER_VAL) {
        state.first_entry = false;
        state.old_pos = ENCODER_VAL % 7;

        LCD_Begin_Payload();
        LCD_Set_Cursor(0, 0);
        LCD_Print_s("Edit values: ");
//        LCD_Print_i(state.old_pos);
        LCD_Set_Cursor(1,0);
        LCD_Print_s("X P I D S I O");
        LCD_Set_Cursor(1, state.old_pos*2);
        LCD_Enable(true, true, false);
        LCD_End_Payload();
        LCD_Send_Payload();
    }
    osDelay(200);
}

void Set_LCD_Edit_Value(const char* c_val, const char* pre_str, const char* suf_str, float val, uint8_t val_width, uint8_t val_prec) {
    static bool old_edit = false;
    if (state.first_entry == true
        || old_edit != state.edit_digit
        || (state.edit_digit == true && state.old_digit != ENCODER_VAL)
        || (state.edit_digit == false && state.old_pos != ENCODER_VAL)) {
        state.first_entry = false;
        old_edit = state.edit_digit;

        if (state.edit_digit == true) {
            state.old_digit = ENCODER_VAL % 10;
        } else {
            state.old_pos = ENCODER_VAL % 16;
        }

        LCD_Begin_Payload();
        LCD_Set_Cursor(0, 0);
        LCD_Print_s(c_val);
        LCD_Print_s(": ");
//        LCD_Print_i(state.old_pos);
        LCD_Set_Cursor(1,0);
        LCD_Print_s(pre_str);
        LCD_Print_ff(val, val_width, val_prec);
        LCD_Print_s(suf_str);
        LCD_Set_Cursor(1, state.old_pos);
        if (state.edit_digit) {
            LCD_Print_c('0' + state.old_digit);
            LCD_Set_Cursor(1, state.old_pos);
            LCD_Enable(true, true, true);
        } else {
            LCD_Enable(true, true, false);
        }
        LCD_End_Payload();
        LCD_Send_Payload();
    }
    osDelay(200);
}

_Noreturn void Service_LCD_Task(__attribute__((unused)) void const* argument) {
    static enum lcd_state old_state = LCD_MAIN;
    static size_t cycle = 0;

    while (1) {
        if (old_state != state.current_lcd || cycle == 20) {
            old_state = state.current_lcd;
            state.first_entry = true;
            cycle = 0;

            LCD_Begin_Payload();
            LCD_Clear_Display();
            LCD_End_Payload();
            LCD_Send_Payload();
        }

        switch (old_state) {
            case LCD_MAIN:
                Set_LCD_Main();
                break;
            case LCD_EDIT:
                Set_LCD_Edit();
                break;
            case LCD_Kp:
                Set_LCD_Edit_Value("Kp", edit_pid_pre, edit_pid_suf, parameters.Kp, PID_VAL_WIDTH, PID_VAL_PREC);
                break;
            case LCD_Ki:
                Set_LCD_Edit_Value("Ki", edit_pid_pre, edit_pid_suf, parameters.Ki, PID_VAL_WIDTH, PID_VAL_PREC);
                break;
            case LCD_Kd:
                Set_LCD_Edit_Value("Kd", edit_pid_pre, edit_pid_suf, parameters.Kd, PID_VAL_WIDTH, PID_VAL_PREC);
                break;
            case LCD_S:
                Set_LCD_Edit_Value("Setpoint", edit_s_pre, edit_s_suf, parameters.S_50, S_VAL_WIDTH, S_VAL_PREC);
                break;
            case LCD_I:
                Set_LCD_Edit_Value("Input Speed", edit_i_pre, edit_i_suf, parameters.I_50, I_VAL_WIDTH, I_VAL_PREC);
                break;
            case LCD_O:
                Set_LCD_Edit_Value("Output Voltage", edit_o_pre, edit_o_suf, parameters.O_50, O_VAL_WIDTH, O_VAL_PREC);
                break;
            default:
                ;
        }

        cycle++;
    }
}

static void Controller_Do_Control(void) {
    static double ITerm = 0;
    static uint32_t lastInput = 0;

    if (state.manual_control) {
        uint16_t val = (uint16_t)floor((float)state.S_RPM * state.C);
        val = CLAMP(val, DUTY_MIN, DUTY_MAX);
        OUTPUT_VAL = val;
        ITerm = val;
    } else {
        ITerm += state.k_i * state.e_ss * state.C;
        ITerm = CLAMP(ITerm, DUTY_MIN, DUTY_MAX);

        double dInput = (state.I_RPM - lastInput)*state.C;
        double output = state.k_p*state.e_ss*state.C + ITerm - state.k_d*dInput;
        OUTPUT_VAL = CLAMP((uint16_t)floor(output), DUTY_MIN, DUTY_MAX);
    }
    lastInput = state.I_RPM;
}

_Noreturn void Service_Control_Task(__attribute__((unused)) void const* argument) {
    static size_t count = 0;
    static float S_avg = 0.0f;
    static float I_avg = 0.0f;
    static int sample_count = SAMPLE_RATE;

    while(1) {
        if (state.adc_done || count++ == 30) {
            count = 0;
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(CPU_LOAD_GPIO_Port, CPU_LOAD_Pin, GPIO_PIN_SET);

            state.S_V = CLAMP((((float) state.adc_buff[0] - ADC_OFFSET) / ADC_CONST), (float) ADC_MIN, (float) ADC_MAX);
            state.I_V = CLAMP((((float) state.adc_buff[1] - ADC_OFFSET) / ADC_CONST), (float) ADC_MIN, (float) ADC_MAX);

            HAL_ADC_Start_DMA(&hadc1, (uint32_t *) state.adc_buff, ADC_BUFF_LEN);
            state.adc_done = false;

            state.S_RPM = (uint16_t) floorf(state.S_V * parameters.S_50 / ADC_MID);
            state.I_RPM = (uint16_t) floorf(state.I_V * parameters.I_50 / ADC_MID);

            if (sample_count < SAMPLE_RATE/16) {
                S_avg = (S_avg + (float)state.S_RPM)/2;
                I_avg = (I_avg + (float)state.I_RPM)/2;
                sample_count++;
            } else {
                *state.S_ptr = (uint16_t)floorf(S_avg);
                *state.I_ptr = (uint16_t)floorf(I_avg);

                sample_count = 0;
                S_avg = state.S_RPM;
                I_avg = state.I_RPM;

                state.S_ptr = (state.S_ptr-state.S_buff)<SCREEN_BUFF_LEN-1 ? state.S_ptr+1 : state.S_buff;
                state.I_ptr = (state.I_ptr-state.I_buff)<SCREEN_BUFF_LEN-1 ? state.I_ptr+1 : state.I_buff;
            }

            state.e_ss = state.S_RPM - state.I_RPM;

            Controller_Do_Control();

            HAL_GPIO_WritePin(CPU_LOAD_GPIO_Port, CPU_LOAD_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        }

        osThreadYield();
    }
}

void Controller_ADC_Done(void) {
    state.adc_done = true;
}

static uint8_t Get_Digit(const float *value, uint8_t val_width, uint8_t val_prec, uint8_t pos) {
    char numstr[10];
    sprintf(numstr, "%0*.*f", val_width, val_prec, *value);
    return numstr[pos] - '0';
}

static void Store_Digit(float* value, uint8_t val_width, uint8_t val_prec, uint8_t digit, uint8_t pos) {
    char numstr[10];
    sprintf(numstr, "%0*.*f", val_width, val_prec, *value);
    numstr[pos] = '0' + digit;

    *value = (float)strtod(numstr, NULL);
}

static void Edit_Value(float* value, uint8_t val_width, uint8_t val_prec, uint8_t pos_min, uint8_t pos_mid, uint8_t pos_max) {
    if (state.old_pos == LCD_X) {
        state.current_lcd = LCD_EDIT;
        ENCODER_RESET();
        ENCODER_MAX(10);
    } else if (state.old_pos < pos_max && state.old_pos > pos_min && state.old_pos != pos_mid) {
        if (state.edit_digit)
            Store_Digit(value, val_width, val_prec, state.old_digit, state.old_pos-pos_min-1);

        state.edit_digit ^= 1;
        state.old_digit = state.edit_digit ? Get_Digit(value, val_width, val_prec, state.old_pos-pos_min-1) : state.old_pos;

        Update_Values();

        TIM2->CNT = state.old_digit << 2;
        ENCODER_MAX(state.edit_digit ? 10 : 16);
    }
}

void Controller_Change_State(void) {
    if (state.enc_cooldown == true)
        return;

    Start_Buzzer();
    ENC_COOLDOWN_STOP();
    state.enc_cooldown = true;
    ENC_COOLDOWN_START();

    switch (state.current_lcd) {
        case LCD_MAIN:
            state.current_lcd = LCD_EDIT;
            ENCODER_RESET();
            ENCODER_MAX(7);
            break;
        case LCD_EDIT:
            if (state.old_pos == LCD_X) {
                state.current_lcd = LCD_MAIN;
                ENCODER_RESET();
                ENCODER_MAX(10);
            } else {
                state.current_lcd = state.old_pos;
                ENCODER_RESET();
                ENCODER_MAX(16);
            }
            break;
        case LCD_Kp:
            Edit_Value(&parameters.Kp, PID_VAL_WIDTH, PID_VAL_PREC, PID_EDIT_MIN, PID_EDIT_MID, PID_EDIT_MAX);
            break;
        case LCD_Ki:
            Edit_Value(&parameters.Ki, PID_VAL_WIDTH, PID_VAL_PREC, PID_EDIT_MIN, PID_EDIT_MID, PID_EDIT_MAX);
            break;
        case LCD_Kd:
            Edit_Value(&parameters.Kd, PID_VAL_WIDTH, PID_VAL_PREC, PID_EDIT_MIN, PID_EDIT_MID, PID_EDIT_MAX);
            break;
        case LCD_S:
            Edit_Value(&parameters.S_50, S_VAL_WIDTH, S_VAL_PREC, S_EDIT_MIN, S_EDIT_MID, S_EDIT_MAX);
            break;
        case LCD_I:
            Edit_Value(&parameters.I_50, I_VAL_WIDTH, I_VAL_PREC, I_EDIT_MIN, I_EDIT_MID, I_EDIT_MAX);
            break;
        case LCD_O:
            Edit_Value(&parameters.O_50, O_VAL_WIDTH, O_VAL_PREC, O_EDIT_MIN, O_EDIT_MID, O_EDIT_MAX);
            break;
        default:
            ;
    }
}

void Controller_Change_Control(void) {
    if (state.btn_cooldown == true)
        return;

    BTN_COOLDOWN_STOP();
    state.btn_cooldown = true;
    BTN_COOLDOWN_START();

    state.manual_control ^= 1;
    HAL_GPIO_WritePin(ControlLED_GPIO_Port, ControlLED_Pin, state.manual_control);
}

void Controller_Reset_Enc_Cooldown(void) {
    state.enc_cooldown = false;
}

void Controller_Reset_Btn_Cooldown(void) {
    state.btn_cooldown = false;
}

_Noreturn void Service_OLED_Task(__attribute__((unused)) void *argument) {
    while (1) {
        // Clear screen
        OLED_Fill(OLED_COLOR_BLACK);

        // Draw divider
        OLED_DrawLine(0, 31, 127, 31, OLED_COLOR_WHITE);
        OLED_DrawLine(0, 32, 127, 32, OLED_COLOR_WHITE);

        // Draw cursor
        uint16_t x = state.S_ptr-state.S_buff;
        OLED_DrawLine(x, 0, x, 63, OLED_COLOR_WHITE);

        bool toggle = true;
        for (int i = 0; i < 127; i++) {
            if (i % 4 == 0)
                toggle ^= 1;

            if (toggle)
                OLED_DrawPixel(i, 48, OLED_COLOR_WHITE);
        }

        int16_t max = 0;
        for (size_t i = 0; i < SCREEN_BUFF_LEN; i++) {
            max = MAX(state.S_buff[i], max);
        }

        uint16_t prev = (uint16_t)(floor(state.S_buff[0] * -1*(30.0/max)) + (int16_t)30);
        for (int i = 1; i < SCREEN_BUFF_LEN; i++) {
            uint16_t val = (uint16_t)(floor(state.S_buff[i] * (-30.0/max)) + (int16_t)30);
            OLED_DrawLine((i-1), prev,i, val, OLED_COLOR_WHITE);

            prev = val;
        }


        max = 0;
        for (size_t i = 0; i < SCREEN_BUFF_LEN; i++) {
            int val = (int16_t)(state.S_buff[i]-state.I_buff[i]);
            val = val < 0 ? (-1 * val) : val;
            max = MAX(val, max);
        }

        prev = (int16_t)(ceil((state.S_buff[0]-state.I_buff[0]) * (-15.0/max)) + (int16_t)48);
        for (int i = 1; i < SCREEN_BUFF_LEN; i++) {
            uint16_t val = (int16_t)(ceil((state.S_buff[i]-state.I_buff[i]) * (-15.0/max)) + (int16_t)48);
            OLED_DrawLine((i-1), prev,i, val, OLED_COLOR_WHITE);

            prev = val;
        }

        OLED_UpdateScreen();
        osDelay(20);
    }
}