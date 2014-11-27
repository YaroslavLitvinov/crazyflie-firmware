#ifndef STABILIZER_REAL_FLY_H
#define STABILIZER_REAL_FLY_H


#define CIRCLE_STABILIZER_PREIOD 3000
#define CIRCLE_STABILIZER_AMPLITUDE_MOTOR 4300
#define CIRCLE_STABILIZER_ACCEPTABLE_AMPLITUDE_RANGE 2000
#define CIRCLE_STABILIZER_MSTIME_TO_ENABLE 3000
#define MOTOR_STABILIZED_VALUE(motorLowerLimitPower, motorUpperLimitPower) \
		((uint32_t)motorLowerLimitPower + abs(motorUpperLimitPower - motorLowerLimitPower)/2)


/*feature enable*/

#define CIRCLE_STABILIZATION_ENABLE


struct motors_power
{
	int timestamp;
	short m1;
	short m2;
	short m3;
	short m4;
};

void CircleStabilizer(short timeDelta,
					  uint32_t *m1, uint32_t *m2, uint32_t *m3, uint32_t *m4);

void assignMotorValuesForCircleMode(short timeDelta, struct motors_power *mPower );

#endif //STABILIZER_REAL_FLY_H
