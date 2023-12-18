//
// Created by Jay on 10/31/2023.
//

#ifndef PRIMROSE_MCIU_POSITIONSENSOR_H
#define PRIMROSE_MCIU_POSITIONSENSOR_H

/**
 * @brief Base class for all position sensors
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
