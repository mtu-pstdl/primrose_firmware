//
// Created by Jay on 10/31/2023.
//

#ifndef PRIMROSE_MCIU_POSITIONSENSOR_H
#define PRIMROSE_MCIU_POSITIONSENSOR_H

class PositionSensor {

    int32_t get_position() const{
        return 0;
    }

    uint32_t get_last_update_time(){
        return 0;
    }

    boolean is_valid(){
        return false;
    }

    boolean fault() const{
        return false;
    }

};

#endif //PRIMROSE_MCIU_POSITIONSENSOR_H
