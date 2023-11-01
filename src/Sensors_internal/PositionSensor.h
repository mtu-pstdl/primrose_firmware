//
// Created by Jay on 10/31/2023.
//

#ifndef PRIMROSE_MCIU_POSITIONSENSOR_H
#define PRIMROSE_MCIU_POSITIONSENSOR_H

class PositionSensor {

public:
    virtual // Implemented only by sensors that don't use the ADAU bus interface
    void update() {

    }

    virtual int32_t get_position() const{
        return 0;
    }

    virtual float_t get_velocity() const{
        return 0;
    }

    virtual uint32_t get_last_update_time(){
        return 0;
    }

    virtual boolean is_valid(){
        return false;
    }

    virtual boolean fault() const{
        return false;
    }

};

#endif //PRIMROSE_MCIU_POSITIONSENSOR_H
