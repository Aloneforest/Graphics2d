#ifndef GRAPHICS2D_CONST2D_H
#define GRAPHICS2D_CONST2D_H

namespace lib2d
{
    const auto inf = std::numeric_limits<double>::infinity();

    //计算类型
    enum UpdateType
    {
        INIT_FORCE_AND_TORQUE,                  //初始化力和力矩
        CALC_VELOCITY_AND_ANGULAR_VELOCITY,     //计算速度和角速度
        CALC_DISPLACEMENT_AND_ANGLE,            //计算位移和角度
        ADD_GRAVITY,                            //添加重力
        RESET_NET_FORCE,                        //重设合外力
        DETERMINE_DORMANCY                      //当合外力和速度为零时，判定休眠
    };

    enum body2dType
    {
        POLYGON,                                    //多边形
        CIRCLE                                      //圆
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
