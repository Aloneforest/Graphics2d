#ifndef GRAPHICS2D_CONST2D_H
#define GRAPHICS2D_CONST2D_H

namespace lib2d
{
    const auto inf = std::numeric_limits<double>::infinity();

    //��������
    enum UpdateType
    {
        INIT_FORCE_AND_TORQUE,                  //��ʼ����������
        CALC_VELOCITY_AND_ANGULAR_VELOCITY,     //�����ٶȺͽ��ٶ�
        CALC_DISPLACEMENT_AND_ANGLE,            //����λ�ƺͽǶ�
        ADD_GRAVITY,                            //�������
        RESET_NET_FORCE                         //���������
    };

    enum body2dType
    {
        POLYGON,                                    //�����
        CIRCLE                                      //Բ
    };
}

#endif //GRAPHICS2D_CONST2D_H
