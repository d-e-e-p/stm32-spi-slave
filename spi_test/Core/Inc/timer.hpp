
// timer functions


void setup_timers(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);


void report_clock_setup(void);
void setup_timer_period(void);
void start_timers(void);
void report_timer_setup(int index, TIM_HandleTypeDef *htim);
void load_timer_params(void);
uint32_t get_timestamp_ms(void);
void print_time(void);

