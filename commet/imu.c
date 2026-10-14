#include "imu.h"
/* ============================ŷ���Ǽ���================================== */
/* ===============================��ʼ===================================== */

/* ����ŷ�����õ���3������ */
float RtA = 57.2957795f; // ����->��
// �����ǳ�ʼ������+-2000��/���� 1/(65536 / 4000) = 0.03051756*2
// float Gyro_G = 0.03051756f * 2;
float Gyro_G = 4000.0 / 65536; // ��/s
// ��ÿ��,ת������ÿ���� 2*0.03051756 * 0.0174533f = 0.0005326*2
// float Gyro_Gr = 0.0005326f * 2;
float Gyro_Gr = 4000.0 / 65536 / 180 * 3.1415926; // ����/s
#define squa(Sq) (((float)Sq) * ((float)Sq))      /* ����ƽ�� */
/**
 * @description: ���ټ��� 1/sqrt(num)
 * @param {float} number
 */
static float Q_rsqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y = number;
    i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (threehalfs - (x2 * y * y)); // 1st iteration ����һ��ţ�ٵ�����
    return y;
}

static float normAccz; /* z���ϵļ��ٶ� */
/**
 * @description: ����mpu��6������, ��ȡ������̬��ŷ����
 * @param {GyroAccel_Struct} *gyroAccel mpu��6������
 * @param {EulerAngle_Struct} *EulerAngle �����õ���ŷ����
 * @param {float} dt �������� (��λs)
 * @return {*}
 */
void IMU_GetEulerAngle(mpu6050_data *gyroAccel,
                              eular *eulerAngle,
                              float dt)
{
    volatile struct V
    {
        float x;
        float y;
        float z;
    } Gravity, Acc, Gyro, AccGravity;

    static struct V GyroIntegError = {0};
    static float KpDef = 0.8f;
    static float KiDef = 0.0003f;
    static Quaternion_Struct NumQ = {1, 0, 0, 0};
    float q0_t, q1_t, q2_t, q3_t;
    // float NormAcc;
    float NormQuat;
    float HalfTime = dt * 0.5f;

    // ��ȡ��Ч��ת�����е���������
    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);
    // ���ٶȹ�һ��
    NormQuat = Q_rsqrt(squa(gyroAccel->accel.accel_x) +
                       squa(gyroAccel->accel.accel_y) +
                       squa(gyroAccel->accel.accel_z));

    Acc.x = gyroAccel->accel.accel_x * NormQuat;
    Acc.y = gyroAccel->accel.accel_y * NormQuat;
    Acc.z = gyroAccel->accel.accel_z * NormQuat;
    // ������˵ó���ֵ
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
    // �������ٶȻ��ֲ������ٶȵĲ���ֵ
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
    // ���ٶ��ںϼ��ٶȻ��ֲ���ֵ
    Gyro.x = gyroAccel->gyro.gyro_x * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x; // ������
    Gyro.y = gyroAccel->gyro.gyro_y * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
    Gyro.z = gyroAccel->gyro.gyro_z * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;

    // һ�����������, ������Ԫ��
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;

    // ��Ԫ����һ��
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
    NumQ.q0 *= NormQuat;
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    /*��������ϵ�µ�Z��������*/
    float vecxZ = 2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3;     /*����(3,1)��*/
    float vecyZ = 2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;     /*����(3,2)��*/
    float veczZ = 1 - 2 * NumQ.q1 * NumQ.q1 - 2 * NumQ.q2 * NumQ.q2; /*����(3,3)��*/

    float yaw_G = gyroAccel->gyro.gyro_z * Gyro_G; // ��Z����ٶ�������ֵ ת��ΪZ�Ƕ�/��      Gyro_G�����ǳ�ʼ������+-2000��ÿ����1 / (65536 / 4000) = 0.03051756*2
    if ((yaw_G > 0.5f) || (yaw_G < -0.5))         // ����̫С������Ϊ�Ǹ��ţ�����ƫ������
    {
        eulerAngle->yaw += yaw_G * dt; // ���ٶȻ��ֳ�ƫ����
    }

    eulerAngle->pitch = asin(vecxZ) * RtA; // ������

    eulerAngle->roll = atan2f(vecyZ, veczZ) * RtA; // �����

    normAccz = gyroAccel->accel.accel_x * vecxZ + gyroAccel->accel.accel_y * vecyZ + gyroAccel->accel.accel_z * veczZ; /*Z�ᴹֱ�����ϵļ��ٶȣ���ֵ��������бʱ��Z����ٶȵ������ͣ����ǵ���������Ӧ�ó���ֵ*/
}

/**
 * @description: ��ȡZ���ϵļ��ٶ� (����Ѿ���б,�ῼ��z���ϼ��ٶȵĺϳ�)
 * @return {*}
 */
float IMU_GetNormAccZ(void)
{
    return normAccz;
}
/* ======================ŷ���Ǽ���================================== */
/* ========================����===================================== */
