// === Just some stuff ===

/*==========
4 Ball Auton
==========*/
void ShootPreloads()
{
  // Clear old data
  datalogClear();

	JumperChange = -1 + SensorValue[Plus1] + SensorValue[Plus2]*2 + SensorValue[Plus3]*3 + SensorValue[Plus4]*4
								 + SensorValue[Plus5]*5 + SensorValue[Plus6]*6 - SensorValue[Minus2]*2 - SensorValue[Minus3]*3
								 - SensorValue[Minus4]*4 - SensorValue[Minus5]*5 - SensorValue[Minus6]*6;
	JumperChange = -1*JumperChange;

	startTask(ControlTask);							//Flywheel velocity control

	clearLCDLine(0);										//Clears LCD

	bLCDBacklight = true;								//LCD backlight

	nMotorEncoder[LeftFront] = 0;
	nMotorEncoder[Fly1] = 0;

	VelocitySet(155,FULLSETTING+JumperChange);								//Sets flywheel to 155 RPM
  Range = 4;

	sleep(2500);												//Accelerate for 2.5 seconds

	MoveLift(UP);

	LastSysTime = nSysTime;							//Save old system time

	int a = 0;
	while((a<3)||(TimeElapsed < 5000))	//Run only for a maximum of 5 seconds (I know, weird time elapsed thing)
	{
		while((FullSpeed == 0)&&(TimeElapsed < 5000))
		{
			MoveFeeder();
			sleep(25);
			TimeElapsed = nSysTime - LastSysTime;
		}
		while((FullSpeed == 1)&&(TimeElapsed < 5000))
		{
			StopFeeder();
			MoveLift(SLOW);
			sleep(25);
			TimeElapsed = nSysTime - LastSysTime;
		}
		wait1Msec(100);
		a++;
	}
	StopFeeder();
	StopLift();
	VelocitySet(0,0);
}

/*=============
RPM Adjustments
=============*/
if(vexRT[Btn7U]==1)					//Sets Flywheel To Full Court
{
	CurrentSettingTarget = 155;
	CurrentSettingPredicted = FULLSETTING+JumperChange;
	VelocitySet(CurrentSettingTarget, CurrentSettingPredicted);
	Adjustment = 0;
	Range = 4;
}

if(vexRT[Btn7R]==1)					//Sets Flywheel To 3/4 Court
{
	CurrentSettingTarget = 125;
	CurrentSettingPredicted = THREEQSETTING+(JumperChange*2/3);
	VelocitySet(CurrentSettingTarget, CurrentSettingPredicted);
	Adjustment = 0;
	Range = 3;
}

if(vexRT[Btn7D]==1)					//Sets Flywheel To 1/2 Court
{
	CurrentSettingTarget = 121;
	CurrentSettingPredicted = HALFSETTING+(JumperChange/2);
	VelocitySet(CurrentSettingTarget, CurrentSettingPredicted);
	Adjustment = 0;
	Range = 2;
}

if(vexRT[Btn8L]==1)					//Sets Flywheel To 1/4 Court
{
	CurrentSettingTarget = 100;
	CurrentSettingPredicted = QUARTERSETTING;
	VelocitySet(CurrentSettingTarget, CurrentSettingPredicted);
	Adjustment = 0;
	Range = 1;
}

if(vexRT[Btn8U]==1)					//Sets Flywheel To 0
{
	CurrentSettingTarget = 0;
	CurrentSettingPredicted = 0;
	VelocitySet(CurrentSettingTarget, CurrentSettingPredicted);
	Adjustment = 0;
	Range = 0;
  }

if(vexRT[Btn8R]==1)
{
	if(vexRT[Btn5U]==1)
	{
		Adjustment = Adjustment + 1;
		wait1Msec(100);
		VelocitySet(CurrentSettingTarget+Adjustment, CurrentSettingPredicted);
	}
	if(vexRT[Btn5D]==1)
	{
		Adjustment = Adjustment - 1;
		wait1Msec(100);
		VelocitySet(CurrentSettingTarget+Adjustment, CurrentSettingPredicted);
	}
}

/*=========================================================
Bang Bang + Some PI+C (Based off of jpearman's TBH control)
=========================================================*/
//Flywheel Definitions
bool FullSpeed;										//Holds Logic if Flywheel is At Full
long FullSpeedStartTime;

int Range;

#define LOOP_SPEED 25							//Update inteval (in mS) for the flywheel control loop

long EncoderCounts;								//Current Encoder Value
long EncoderCountsLast;						//Last Encoder Value

float MotorVelocity;							//Current Velocity in RPM
long encoder_timestamp;
long encoder_timestamp_last;			//Time of Last Velocity Calculation

long TargetVelocity;       				//Target RPM

float CurrentError;      			    //Current Error Between Target and Actual RPMs
float LastError;          			  //Last Error
float AverageError;								//Average of Current Error and Last Error

float FilteredError;							//Average of Current and Last Three Errors
float BatteryAdjustmentFull;			//Adjustment of Predicted Motor Power For Full
float BatteryAdjustmentThreeQ;		//								 "									For Three Quarters
float BatteryAdjustmentHalf;			//								 "									For Half
float BatteryAdjustmentQuarter;		//								 "									For Quarter
float BatteryAdjustment;					//								 "									In Current Setting

float ProportionalFw;
float IntegralFw;
float kP_Fw = 1.5;
float kI_Fw = .7;
bool PIControlOn = false;

int Predicted;										//Predicted Motor Value

void Flywheel(int Speed)					//Sets Flywheel Speed
{
	motor[Fly1] = Speed;
	motor[Fly2] = Speed;
	motor[Fly3] = Speed;
}

void CalculateSpeed()
{
	int delta_ms;
	int delta_enc;

  // Get current encoder value and time
  getEncoderAndTimeStamp(Fly1, EncoderCounts, encoder_timestamp);

  // This is just used so we don't need to know how often we are called
  // how many mS since we were last here
  delta_ms = encoder_timestamp - encoder_timestamp_last;
  encoder_timestamp_last = encoder_timestamp;

	//Change in encoder count
	delta_enc = (EncoderCounts - EncoderCountsLast);

	//Save last position
	EncoderCountsLast = EncoderCounts;

	//Calculate velocity in rpm
	MotorVelocity = (1000.0 / (delta_ms+0.0001)) * delta_enc * 60.0 / 392;
}

void ControlUpdateVelocity()
{
	// calculate error in velocity
	// target_velocity is desired velocity
	// current is measured velocity
	LastError = CurrentError;
	CurrentError	= TargetVelocity - MotorVelocity;
	AverageError = LastError*.5 + CurrentError*.5;

	ProportionalFw = AverageError * kP_Fw;
	IntegralFw = (IntegralFw + AverageError) * kI_Fw;

	if(CurrentError > 20)
	{
		IntegralFw = 0;
		kP_Fw = 1.5;
	}
	if(CurrentError < 5)
 {
		kP_Fw = .7;
	}

	FilteredError = CurrentError*.25 + LastError*.75;		//This is for automatic adjustment
}

task ControlTask()
{
	while(1)
	{
		// Calculate velocity
		CalculateSpeed();

		ControlUpdateVelocity();

		// and finally set the motor control value
		// Keeps previous value if CurrentError is between 5 and 20
		if(Range == 0)
		{
			Flywheel(0);
		}

		if((Range == 1)||(Range == 2)||(Range == 3))
		{
			if(FullSpeed)
			{
				FullSpeedStartTime = nSysTime;
			}
			if(AverageError<5)
			{
				FullSpeed = false;
				if(Range==3)
				{
					Flywheel(Predicted+BatteryAdjustmentThreeQ);
				}
				if(Range==2)
				{
					Flywheel(Predicted+BatteryAdjustmentHalf);
				}
				if(Range==1)
				{
					Flywheel(Predicted+BatteryAdjustmentQuarter);
				}
			}
		}

		else if(Range>3)
		{
			if(PIControlOn)
			{
				FullSpeed = false;
				Flywheel(ProportionalFw+IntegralFw+Predicted);
			}
			else
			{
				if(FullSpeed)
				{
					FullSpeedStartTime = nSysTime;
				}
				if(AverageError<5)
				{
					FullSpeed = false;
					Flywheel(Predicted+BatteryAdjustmentFull);
				}
			}
		}

		if(AverageError>20)
		{
			Flywheel(127);
			FullSpeed = true;
		}

  	if((!FullSpeed)&&(!Range==0))
  	{
  		if((nSysTime-FullSpeedStartTime)>1000)
  		{
  			if(Range == 4)
  			{
					if(FilteredError>5)
					{
						BatteryAdjustmentFull += .2;
						BatteryAdjustment = BatteryAdjustmentFull;
					}
					else if(FilteredError<(-5))
					{
						BatteryAdjustmentFull -= .2;
						BatteryAdjustment = BatteryAdjustmentFull;
					}
  			}
  			else if(Range == 3)
  			{
					if(FilteredError>5)
					{
						BatteryAdjustmentThreeQ += .1;
						BatteryAdjustment = BatteryAdjustmentThreeQ;
					}
					else if(FilteredError<(-5))
					{
						BatteryAdjustmentThreeQ -= .1;
						BatteryAdjustment = BatteryAdjustmentThreeQ;
					}
  			}
  			else if(Range == 2)
  			{
					if(FilteredError>5)
					{
						BatteryAdjustmentHalf += .1;
						BatteryAdjustment = BatteryAdjustmentHalf;
					}
					else if(FilteredError<(-5))
					{
						BatteryAdjustmentHalf -= .1;
						BatteryAdjustment = BatteryAdjustmentHalf;
					}
  			}
  			else if(Range == 1)
  			{
					if(FilteredError>5)
					{
						BatteryAdjustmentQuarter += .1;
						BatteryAdjustment = BatteryAdjustmentQuarter;
					}
					else if(FilteredError<(-5))
					{
						BatteryAdjustmentQuarter -= .1;
						BatteryAdjustment = BatteryAdjustmentQuarter;
					}
  			}
			}
		}

		// Run at somewhere between 20 and 50mS
		wait1Msec(LOOP_SPEED);
	}
}
