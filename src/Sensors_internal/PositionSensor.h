//
// Created by Jay on 10/31/2023.
//

#ifndef PRIMROSE_MCIU_POSITIONSENSOR_H
#define PRIMROSE_MCIU_POSITIONSENSOR_H

/**
 * PositionSensor is an abstract class that provides an interface for all position sensors to implement
 * @details Position sensors provide the position and velocity of a sensor
 * @warning Do not instantiate this class directly. Instead use one of the derived classes
 */
class PositionSensor {

public:

    virtual void initialize() {}

    virtual void update() {}

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

    virtual uint8_t fault() {
        return false;
    }

};

#endif //PRIMROSE_MCIU_POSITIONSENSOR_H
