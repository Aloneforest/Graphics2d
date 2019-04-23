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
        RESET_NET_FORCE,                        //���������
        DETERMINE_DORMANCY                      //�����������ٶ�Ϊ��ʱ���ж�����
    };

    enum body2dType
    {
        POLYGON,                                    //�����
        CIRCLE                                      //Բ
    };

    enum color 
    {
        YELLOW,
        RED,
        WHITE
    };

    template<typename ContainerT, typename PredicateT>
    void erase_if(ContainerT &items, const PredicateT &predicate) {
        for (auto it = items.begin(); it != items.end();) {
            if (predicate(*it)) it = items.erase(it);
            else ++it;
        }
    };
}

#endif //GRAPHICS2D_CONST2D_H
