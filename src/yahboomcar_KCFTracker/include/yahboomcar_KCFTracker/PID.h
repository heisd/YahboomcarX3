
class PID {
public:
    float kp;
    float ki;
    float kd;
    float targetpoint = 0.0f;
    // ��һ�ε����
    //Last time error
    float prevError = 0.0f;
    // ����
    //integral
    float intergral = 0.0f;
    // ΢��
    //differential
    float derivative = 0.0f;

    PID(float kp, float ki, float kd);

    void Set_PID(float kp, float ki, float kd);

    /**
     * pid calculation function pid�ļ��㺯��
     * @param target  Ŀ��ֵ
     * @param current ��ǰֵ
     * @return  pwm
     */
    float compute(float target, float current);

    /**
     *  �������е����: �����õ��ٶ� �� ��һ�β�һ��
     *  Reset all errors: When the set speed is different from the last time
     */
    void reset();
};
