#include <ArduinoHardware.h>
#include <ros.h>
#include <math.h>

#include <std_msgs/Float32MultiArray.h>

#define NUM_WHEELS 4
#define VELOCITY_EPS 0.01f
#define MAX_VELOCITY 12.0f      //rad/s
#define PAUSE_TIME 150          //ms

#define WHEEL_LEFT_FRONT_PWM    10
#define WHEEL_LEFT_FRONT_A      9
#define WHEEL_LEFT_FRONT_B      8

#define WHEEL_LEFT_BACK_PWM     11
#define WHEEL_LEFT_BACK_A       12
#define WHEEL_LEFT_BACK_B       13

#define WHEEL_RIGHT_FRONT_PWM   5
#define WHEEL_RIGHT_FRONT_A     6
#define WHEEL_RIGHT_FRONT_B     7

#define WHEEL_RIGHT_BACK_PWM    3
#define WHEEL_RIGHT_BACK_A      2
#define WHEEL_RIGHT_BACK_B      4

ros::NodeHandle_<ArduinoHardware, 1, 1, 128, 128> g_NodeHandle;

void velocitiesCallback(const std_msgs::Float32MultiArray&);
ros::Subscriber<std_msgs::Float32MultiArray> g_Subscriber("/wheel_velocities", &velocitiesCallback);

bool g_ParamsFetched{ false };
bool g_PrintWheelVelocities{ false };
int g_FixedPwm{ -1 };



class Pin
{
public:
    Pin(uint8_t pinId, bool isAnalog = false) 
        : m_Id(pinId)
        , m_IsAnalog(isAnalog)
    {}

    void Setup()
    {
        m_Value = LOW;
        pinMode(m_Id, OUTPUT);
        Write();
    }   

    void SetValue(uint8_t value)
    {
        if(m_Value == value)
        {
            return;
        }

        m_Value = value;
        m_Dirty = true;
    }

    uint8_t GetValue() const { return m_Value; }

    void Write() 
    {
        if(!m_Dirty)
        {
            return;
        }

        if(m_IsAnalog)
        {
            analogWrite(m_Id, m_Value);
        }
        else
        {
            digitalWrite(m_Id, m_Value);
        }

        m_Dirty = false;
    }

    bool IsDirty() const { return m_Dirty; }

private:
    uint8_t m_Id;
    uint8_t m_Value{ LOW };
    bool m_IsAnalog{ false };
    bool m_Dirty{ false };
};

class Wheel
{
public:
    Wheel(uint8_t pinPwm, uint8_t pinA, uint8_t pinB) 
        : m_PinPwm(pinPwm, true) 
        , m_PinA(pinA)
        , m_PinB(pinB)
        {}
    
    void Setup()
    {
        m_PinPwm.Setup();
        m_PinA.Setup();
        m_PinB.Setup();
    }

    void Loop()
    {
        unsigned long now = millis();
        m_Dirty = false;

        if(m_IsPaused)
        {
            if(m_CurrentDirection * m_WantedVelocity > 0.0f)
            {
                m_PauseTimestamp = now;
                m_IsPaused = false;
            }
            else
            {
                if(now >= m_PauseTimestamp)
                {
                    m_CurrentDirection = 0.0f;
                }
                else
                {
                    return;
                }
            }
        }

        if(m_CurrentDirection * m_WantedVelocity < 0.0f)
        {
            m_WantedVelocity = 0.0f;
        }
        
        ConvertVelocityToPinValues(m_WantedVelocity, m_PinPwm, m_PinA, m_PinB);
        
        m_Dirty = (m_PinPwm.IsDirty() || m_PinA.IsDirty() || m_PinB.IsDirty());

        m_PinPwm.Write();
        m_PinA.Write();
        m_PinB.Write();

        if(m_WantedVelocity != 0.0f)
        {
            m_CurrentDirection = (0.0f < m_WantedVelocity) - (m_WantedVelocity < 0.0f);
        }
        else if(m_CurrentDirection != 0)
        {
            m_PauseTimestamp = now + PAUSE_TIME;
            m_IsPaused = true;
        }
    }

    void Stop()
    {
        m_WantedVelocity = 0.0f;
        m_CurrentDirection = 0;

        m_PinPwm.SetValue(0);
        m_PinPwm.SetValue(LOW);
        m_PinPwm.SetValue(LOW);

        m_PinPwm.Write();
        m_PinA.Write();
        m_PinB.Write();
    }

    void SetWantedVelocity(float velocity) 
    { 
        m_WantedVelocity = velocity;
        if(fabs(m_WantedVelocity) < VELOCITY_EPS)
        {
            m_WantedVelocity = 0.0f;
        }

        m_WantedVelocity = min(m_WantedVelocity, MAX_VELOCITY);
    }

    bool IsDirty() const { return m_Dirty; }

    void DumpDebugString(char (&buffer)[8]) const
    {
        sprintf(buffer, "%d(%d%d)", m_PinPwm.GetValue(), m_PinA.GetValue(), m_PinB.GetValue());
    }

private:
    static void ConvertVelocityToPinValues(float velocity, Pin& pwm, Pin& a, Pin& b)
    {
        if(velocity < -VELOCITY_EPS)
        {
            a.SetValue(HIGH);
            b.SetValue(LOW);
        }
        else if(velocity > VELOCITY_EPS)
        {
            a.SetValue(LOW);
            b.SetValue(HIGH);
        }
        else
        {
            a.SetValue(LOW);
            b.SetValue(LOW);
        }

        pwm.SetValue( g_FixedPwm > 0 ? (velocity != 0.0f ? g_FixedPwm : 0) : ((fabs(velocity) / MAX_VELOCITY) * 255) );
    }

private:
    Pin m_PinPwm;
    Pin m_PinA;
    Pin m_PinB;

    float m_WantedVelocity{ 0.0f };
    int8_t m_CurrentDirection{ 0 };

    unsigned long m_PauseTimestamp{ 0 };
    bool m_IsPaused{ false };
    bool m_Dirty{ false };
};

Wheel m_Wheels[NUM_WHEELS]
{
    {WHEEL_LEFT_FRONT_PWM,    WHEEL_LEFT_FRONT_A,     WHEEL_LEFT_FRONT_B  },
    {WHEEL_LEFT_BACK_PWM,     WHEEL_LEFT_BACK_A,      WHEEL_LEFT_BACK_B   },
    {WHEEL_RIGHT_FRONT_PWM,   WHEEL_RIGHT_FRONT_A,    WHEEL_RIGHT_FRONT_B },
    {WHEEL_RIGHT_BACK_PWM,    WHEEL_RIGHT_BACK_A,     WHEEL_RIGHT_BACK_B  }
};





void velocitiesCallback(const std_msgs::Float32MultiArray& message)
{
    for(uint8_t i = 0; i < NUM_WHEELS; i++)
    {
        m_Wheels[i].SetWantedVelocity(message.data[i]);
    }
}

void setup()
{
    g_NodeHandle.initNode();
    g_NodeHandle.subscribe(g_Subscriber);
    
    for(Wheel& wheel : m_Wheels)
    {
        wheel.Setup();
    }
}

void fetchParams()
{
    if(!g_NodeHandle.getParam("~printWheelVelocities", &g_PrintWheelVelocities))
    {
        g_PrintWheelVelocities = false;
    }

    if(!g_NodeHandle.getParam("~fixedPwm", &g_FixedPwm))
    {
        g_FixedPwm = 0;
    }
    else
    {
        g_FixedPwm = min(255, max(0, g_FixedPwm));
    }
}

void loop()
{
    while(!g_NodeHandle.connected())
    {
        for(Wheel& wheel : m_Wheels)
        {
            wheel.Stop();
        }

        g_NodeHandle.spinOnce();
    }

    if(!g_ParamsFetched)
    {
        fetchParams();
        g_ParamsFetched = true;
    }

    bool atLeastOneDirty = false;
    for(Wheel& wheel : m_Wheels)
    {
        wheel.Loop();

        atLeastOneDirty = atLeastOneDirty || wheel.IsDirty();
    }

    if(g_PrintWheelVelocities && atLeastOneDirty)
    {
        char debugBuffer[40] = "[";

        for(int i = 0; i < NUM_WHEELS; i++)
        {
            char wheelBuffer[8];

            m_Wheels[i].DumpDebugString(wheelBuffer);
            strcat(debugBuffer, wheelBuffer);

            if(i < NUM_WHEELS - 1)
            {
                strcat(debugBuffer, " ");
            }
        }

        strcat(debugBuffer, "]");

        g_NodeHandle.loginfo(debugBuffer);
    }

    g_NodeHandle.spinOnce();
    delay(1);
}