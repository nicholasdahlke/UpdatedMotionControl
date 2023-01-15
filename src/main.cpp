#include <Arduino.h>
#define MIN_STEP_LENGTH 8
#define MIN_STEP_LENGTH_S 0.000008
#define F_PI 3.1415926535897932384626433832795f

const uint8_t step_pin = 11;
const uint8_t dir_pin = 10;
constexpr float controller_steps = 400.0f;
constexpr float ratio = 5.0f;
constexpr float steps_per_rev = controller_steps * ratio;

// float max_val = 30.0f;
// float period = 2.0f;
constexpr float max_val = 20.0f * (F_PI / 180.0f);
const float period = 0.1;

float manual_angle = 10.0f * (F_PI / 180.0f);
float manual_time = 1.0f;

unsigned long last_step_time = 0;

bool motor_running = false;

const int table_length = floorf(max_val / (2*F_PI/(steps_per_rev))) * 4.0;
float step_width_table[table_length] = {0};
char dir_table[table_length] = {0};


template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

float alpha(float n)
{
   return  n * ((2.0*F_PI) / steps_per_rev);
}

void calculateTimeTable(float period, float max_angle)
{
    float t = 0;
    float c1 = period / (steps_per_rev * max_val);
    Serial.println("Started calculating table");
    for (int i = 0; i < table_length; i++)
    {
        step_width_table[i] = (c1 * (1 / cos(((2 * F_PI) / period) * t))) - MIN_STEP_LENGTH_S;
        dir_table[i] = sgn(step_width_table[i]);
        step_width_table[i] = abs(step_width_table[i]);
        t += step_width_table[i] + MIN_STEP_LENGTH_S;
        Serial.println(step_width_table[i], 6);
    }
    Serial.println("finished");
}

void calculateTimeTable()
{
    for (int i = 0; i < table_length / 4; i++) // Generate first quarter
    {
        step_width_table[i] = asinf(alpha(float(i)) / alpha(float(table_length / 4))) * period / (2*F_PI);
        dir_table[i] = 1;
    }

    for (int i = table_length / 4; i > 0 ; i--)
    {
        step_width_table[i] = step_width_table[i] - step_width_table[i - 1] - MIN_STEP_LENGTH_S;
        if(step_width_table[i] < 0)
            step_width_table[i] = 0;
    }
    
    {
        int j = table_length / 2 - 1;
        for (int i = 0; i <= table_length / 4; i++)
        {
            step_width_table[j] = step_width_table[i];
            dir_table[j] = -1;
            j--;
        }
    }

    for (int i = 0; i < table_length / 2; i++)
    {
        step_width_table[i + table_length / 2] = step_width_table[i];
        dir_table[i + table_length / 2] = -dir_table[i];
    }
    
}

/*
unsigned long getTimeFromTable(float t, float period)
{
    float t_div = t / period;
    float t_reduced = (t_div - floorf(t_div)) * period;
    int i = static_cast<int>(roundf(t_reduced / dt));
    return static_cast<unsigned long>(step_width_table[i] * 1e6f);
}

int getDirFromTable(float t, float period)
{
    float t_div = t / period;
    float t_reduced = (t_div - floorf(t_div)) * period;
    int i = static_cast<int>(roundf(t_reduced / dt));
    return dir_table[i];
}*/

void move(float angle)
{
    int steps_to_go = steps_per_rev * (angle / (2*F_PI));
    float step_time_delay_s = manual_time * ((2*F_PI) / (steps_per_rev * manual_angle));
    unsigned long step_time_delay = static_cast<unsigned long>(step_time_delay_s);
    Serial.println(steps_to_go > 0 ? HIGH : LOW);
    for (int i = 0; i < steps_to_go; i++)
    {
        digitalWrite(dir_pin, steps_to_go > 0 ? HIGH : LOW);
        digitalWrite(step_pin, HIGH);
        delayMicroseconds(MIN_STEP_LENGTH);
        digitalWrite(step_pin, LOW);
        delayMicroseconds(step_time_delay);
    }
}

void setup()
{
    pinMode(step_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    Serial.begin(1000000);
    delay(100);
    calculateTimeTable();
    //calculateDirTable(period, max_val);
    int positive = 0;
    int negative = 0;

    for (int i = 0; i < table_length / 2; i++)
    {
        Serial.println(step_width_table[i], 7);
    }
    

    for (int i = 0; i < table_length; i++)
    {
        if(dir_table[i] > 0)
            positive++;
        else
            negative++;
    }
    Serial.print("positiv: ");
    Serial.println(positive);
    Serial.print("negative");
    Serial.println(negative);
    

}

void handleSerial(uint8_t buffer)
{
    if (buffer == 99) // Turn on motor
        motor_running = true;

    if (buffer == 100) // Turn off motor
        motor_running = false;

    if(buffer == 103)
        manual_angle = Serial.parseFloat() * (F_PI / 180.0f);;

    if(buffer == 'e')
        move(manual_angle);
    
    if (buffer == 'f')
        move(-manual_angle);

    delay(500);
}



constexpr float step_angle = (2.0 * F_PI) / steps_per_rev;
float pos = 0;
int index = 0;

void loop()
{
    // if(true)
    if (motor_running)
    {
        //static unsigned long shift = micros();
        //float time = (micros() - shift) / 1e6;
        //float c1 = period / (steps_per_rev * max_val);
        //float vel = ((2 * PI * max_val) / period) * cos(((2 * PI) / period) * time);
        //float soll_pos = max_val * sin(((2 * PI) / period) * time);
        //float k = 0.0005;
        //float step_time_f = (c1 * (1 / cos(((2 * F_PI) / period) * time))) - MIN_STEP_LENGTH_S;
        //step_time_f = abs(step_time_f);
        //Serial.println(k * sgn(atan2f(pos, vel) - atan2f(soll_pos, vel)) * abs(pos - soll_pos), 8);
        //step_time_f += k * sgn(atan2f(pos, vel) - atan2f(soll_pos, vel)) * abs(pos - soll_pos);
        //unsigned long step_time = static_cast<unsigned long>(step_time_f * 1e6f);
        unsigned long step_time = step_width_table[index] * 1e6f;
        int vel = dir_table[index];
        if ((micros() - last_step_time) >= step_time)
        {
            digitalWrite(dir_pin, vel > 0 ? HIGH : LOW);
            pos += vel > 0 ? step_angle : -step_angle;
            digitalWrite(step_pin, HIGH);
            delayMicroseconds(MIN_STEP_LENGTH);
            digitalWrite(step_pin, LOW);
            last_step_time = micros();
            index++;
            if(index == table_length)
                index = 0;
        }
        Serial.println(pos * ((180)/F_PI));
    }
    else
    {
        if (Serial.available() > 0)
        {
            uint8_t ser_buffer = Serial.read();
            if (ser_buffer != 10) // Linefeed
                handleSerial(ser_buffer);
        }
    }
}