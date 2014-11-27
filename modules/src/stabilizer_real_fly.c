#include "stabilizer_real_fly.h"

#include "debug.h"
#include "math.h"
#include "stdlib.h"


uint32_t timeTryingHover=0;
uint32_t motorLowerLimitPowerM4=0;
uint32_t motorLowerLimitPowerM2=0;
uint32_t motorLowerLimitPowerM1=0;
uint32_t motorLowerLimitPowerM3=0;

uint32_t motorUpperLimitPowerM4=0;
uint32_t motorUpperLimitPowerM2=0;
uint32_t motorUpperLimitPowerM1=0;
uint32_t motorUpperLimitPowerM3=0;

static struct motors_power gCircleFlyingMotorsPowerArray[] =
{
		{165515,48187,42228,45739,39406},
		{165615,46478,38590,44574,39310},
		{165715,46940,40660,44306,36546},
		{165815,43827,36325,40423,34921},
		{165915,47161,37678,42373,38320},
		{166015,49011,36070,39649,38338},
		{166115,42817,38242,42419,32710},
		{166215,48955,41359,45449,40253},
		{166315,48136,39636,42984,38756},
		{166415,44616,37637,40670,32817},
		{166515,42736,36176,42442,35562},
		{166615,46233,41742,45333,36012},
		{166715,50238,40738,44654,39882},
		{166815,47509,42503,48049,39359},
		{166915,48307,40478,43285,38246},
		{167015,44706,35557,40866,36319},
		{167115,43603,32551,36847,34511},
		{167215,43534,35202,42072,36456},
		{167415,45604,39087,43980,37501},
		{167515,44145,37810,39699,34254},
		{167615,44082,35833,39672,34585},
		{167715,44045,38201,42259,34763},
		{167815,41637,36983,42905,34175},
		{167915,41059,38706,42889,33122},
		{168015,43247,36625,41587,34565},
		{168115,41909,40806,45341,33328}
};


#define ARRAY_ITEMS_COUNT (sizeof(gCircleFlyingMotorsPowerArray)/sizeof(struct motors_power))

static int CheckSetMotorLimits(uint32_t *motorLowerLimit, uint32_t *motorUpperLimit,
		                        uint32_t motorPower, uint32_t timeDelta)
{
	/*set and check lower power limits for motors*/
	if ( *motorLowerLimit == 0 || motorPower < *motorLowerLimit  )
	{
		*motorLowerLimit = motorPower;
		timeTryingHover = 0;
	}
	/*set and check upper power limits for motors*/
	else if ( *motorUpperLimit == 0 || motorPower > *motorUpperLimit  )
	{
		*motorUpperLimit = motorPower;
		timeTryingHover = 0;
	}
	/*hovering mode*/
	else
	{
		if ( CIRCLE_STABILIZER_ACCEPTABLE_AMPLITUDE_RANGE < abs(motorUpperLimit - motorLowerLimit)  )
		{
			timeTryingHover = 0;
		}
		else
		{
			timeTryingHover += timeDelta;
		}
	}

	if ( timeTryingHover == 0 )
		return 0;
	else
		return 1;
}


/*moved to bss from transitionMotorValuesInRange due to insufficient stack size*/
static short passedTransitionTime=0;
static short m1_delta=0;
static short m2_delta=0;
static short m3_delta=0;
static short m4_delta=0;

static int transitionMotorValuesInRange(int transitionPeriod,
			short timeResolution,
			struct motors_power *motorsPower,
			const struct motors_power *toMotorsPower)
{
	if ( passedTransitionTime == 0 )
	{
		m1_delta = abs( motorsPower->m1 - toMotorsPower->m1) / timeResolution;
		m2_delta = abs( motorsPower->m2 - toMotorsPower->m2) / timeResolution;
		m3_delta = abs( motorsPower->m3 - toMotorsPower->m3) / timeResolution;
		m4_delta = abs( motorsPower->m4 - toMotorsPower->m4) / timeResolution;
	}

	motorsPower->m1 = motorsPower->m1 < toMotorsPower->m1
			? motorsPower->m1 + m1_delta : motorsPower->m1 - m1_delta;
	motorsPower->m2 = motorsPower->m2 < toMotorsPower->m2
			? motorsPower->m2 + m2_delta : motorsPower->m2 - m2_delta;
	motorsPower->m3 = motorsPower->m3 < toMotorsPower->m3
			? motorsPower->m3 + m3_delta : motorsPower->m3 - m3_delta;
	motorsPower->m4 = motorsPower->m4 < toMotorsPower->m4
			? motorsPower->m4 + m4_delta : motorsPower->m4 - m4_delta;

	passedTransitionTime += timeResolution;
	if ( passedTransitionTime >= transitionPeriod )
	{
		passedTransitionTime = 0;
		return 1;
	}
	else
		return 0;
}

#ifdef CIRCLE_STABILIZATION_ENABLE
static uint32_t CircleStabilizerModeMotorValue(float phase, uint32_t motorStabilizedValue)
{
	float res = CIRCLE_STABILIZER_AMPLITUDE_MOTOR*sin(2*M_PI*timeTryingHover/CIRCLE_STABILIZER_PREIOD + phase)
		+ motorStabilizedValue;
	return (uint32_t)res;
}
#endif

void assignMotorValuesForCircleMode(short timeDelta, struct motors_power *mPower )
{
	//struct motors_power toMotorsPower;
	/*just calculate propagating motors value*/
//	toMotorsPower.m4 = CircleStabilizerModeMotorValue(M_PI*7/4,
//				MOTOR_STABILIZED_VALUE(motorLowerLimitPowerM4, motorUpperLimitPowerM4) );
//	toMotorsPower.m2 = CircleStabilizerModeMotorValue(M_PI*3/4,
//						MOTOR_STABILIZED_VALUE(motorLowerLimitPowerM2, motorUpperLimitPowerM2) );
//	toMotorsPower.m1 = CircleStabilizerModeMotorValue(0,
//						MOTOR_STABILIZED_VALUE(motorLowerLimitPowerM1, motorUpperLimitPowerM1) );
//	toMotorsPower.m3 = CircleStabilizerModeMotorValue(M_PI,
//						MOTOR_STABILIZED_VALUE(motorLowerLimitPowerM3, motorUpperLimitPowerM3) );
	mPower->m4 = CircleStabilizerModeMotorValue(M_PI*7/4, mPower->m4 );
	mPower->m2 = CircleStabilizerModeMotorValue(M_PI*3/4, mPower->m2 );
	mPower->m1 = CircleStabilizerModeMotorValue(0, mPower->m1 );
	mPower->m3 = CircleStabilizerModeMotorValue(M_PI, mPower->m3 );


//	transitionMotorValuesInRange(
//			abs(motorUpperLimitPowerM4 - motorLowerLimitPowerM4) /*ms*/,
//			timeDelta, mPower, &toMotorsPower);
}


static void assignMotorValuesFromStore(short timeDelta, struct motors_power *mPower )
{
	/*moved to bss from assignMotorValuesFromStore due to insufficient stack size*/
	static short index=0;
	static int intermediateTimestamp=0;

	short prevTransitionComplete=0;
	struct motors_power *m = &gCircleFlyingMotorsPowerArray[index];
	if ( index == 0 || index >= ARRAY_ITEMS_COUNT )
	{
		/*organize cyclic loop*/
		index = 0;
		m = &gCircleFlyingMotorsPowerArray[index];
		intermediateTimestamp = m->timestamp;
		prevTransitionComplete = 0;
	}
	else
		intermediateTimestamp += timeDelta;

	if ( !prevTransitionComplete &&
		 1 == transitionMotorValuesInRange( 2000 /*ms*/, timeDelta,
								mPower, m) )
	{
		prevTransitionComplete = 1;
	}
	else
		return;

	if ( index+1 < ARRAY_ITEMS_COUNT  )
	{
		/*linear propogating for motor values*/
		struct motors_power *m_next = &gCircleFlyingMotorsPowerArray[index+1];

		if ( 1 == transitionMotorValuesInRange( abs( m->timestamp - m_next->timestamp ),
					timeDelta,
					mPower, m_next) )
		{
			index++;
		}
	}
}


void CircleStabilizer(short timeDelta,
		uint32_t *m1, uint32_t *m2, uint32_t *m3, uint32_t *m4)
{
	char hoverModePreviouslyEnabled=1;
	if ( timeTryingHover < CIRCLE_STABILIZER_MSTIME_TO_ENABLE )
		hoverModePreviouslyEnabled = 0;

	/*If not yet entered hover mode, check it now, or enter or back to normal mode
	 * and counting timeTryingHover*/
	char p4 = CheckSetMotorLimits( &motorLowerLimitPowerM4, &motorUpperLimitPowerM4, *m4, 0);
	char p2 = CheckSetMotorLimits( &motorLowerLimitPowerM2, &motorUpperLimitPowerM2, *m2, 0);
	char p1 = CheckSetMotorLimits( &motorLowerLimitPowerM1, &motorUpperLimitPowerM1, *m1, 0);
	char p3 = CheckSetMotorLimits( &motorLowerLimitPowerM3, &motorUpperLimitPowerM3, *m3, timeDelta);

	if ( hoverModePreviouslyEnabled && (!p1 || !p2 || !p3 || !p4) )
	{
		DEBUG_PRINT("Circle stabilization mode exit!\n");
	}

	/*If entered into hover mode, stabilize motors function*/
	if ( timeTryingHover >= CIRCLE_STABILIZER_MSTIME_TO_ENABLE
		&& p1 && p2 && p3 && p4 )
	{
		if ( !hoverModePreviouslyEnabled )
		{
			DEBUG_PRINT("Circle stabilization mode entered!\n");
			DEBUG_PRINT("powerM1 [%d] \n", (int)*m1 );
			DEBUG_PRINT("powerM2 [%d] \n", (int)*m2 );
			DEBUG_PRINT("powerM2 [%d] \n", (int)*m3 );
			DEBUG_PRINT("powerM3 [%d] \n", (int)*m4 );
		}

		if ( timeTryingHover >= 4*CIRCLE_STABILIZER_MSTIME_TO_ENABLE )
			timeTryingHover = 0;

		/*linear propogating for motor values*/
		struct motors_power motorsTmp;
		motorsTmp.m1 = *m1;
		motorsTmp.m2 = *m2;
		motorsTmp.m3 = *m3;
		motorsTmp.m4 = *m4;

#ifdef CIRCLE_STABILIZATION_ENABLE
		 assignMotorValuesForCircleMode(timeDelta, &motorsTmp );
#else
		assignMotorValuesFromStore( timeDelta, &motorsTmp );
#endif
		*m1 = motorsTmp.m1;
		*m2 = motorsTmp.m2;
		*m3 = motorsTmp.m3;
		*m4 = motorsTmp.m4;
		//DEBUG_PRINT("m1=%d, m2=%d, m3=%d, m4=%d\n",
		//		(int)*m1, (int)*m2, (int)*m3, (int)*m4);
	}
}




