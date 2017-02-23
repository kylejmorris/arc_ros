/**
* CLASS: MotorSchema
* DATE: 20/02/17
* AUTHOR: Kyle Morris
* DESCRIPTION: interface for any motor schema.
*/
#ifndef ARC_BEHAVIOUR_MOTORSCHEMA_H
#define ARC_BEHAVIOUR_MOTORSCHEMA_H

namespace arc_behaviour {
    class MotorSchema {
        protected:
            bool enabled; //whether or not the motor schema is enabled.

        public:
            //enable or disable the schema. This will prevent it from doing anything. pass in the state to set schema to.
            //true means it is enabled, false is disabled.
            virtual void toggle(bool state) = 0;
    };
}

#endif //ARC_BEHAVIOUR_MOTORSCHEMA_H
