package com.stormbots.closedloop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Small, easy to use PID implementation with advanced controller
 * capability.<br>
 * Minimal usage:<br>
 * MiniPID pid = new MiniPID(p,i,d); <br>
 * ...looping code...{ <br>
 * output= pid.getOutput(sensorvalue,target); <br>
 * }
 * 
 * @see https://github.com/tekdemo/MiniPID-Java
 * @see http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/improving-the-beginners-pid-introduction
 */
public class MiniPID{
	//**********************************
	// Class private variables
	//**********************************

	private double P=0;
	private double I=0;
	private double D=0;
	private double F=0;

	private double maxIOutput=0;
	private double maxError=0;
	private double errorSum=0;

	private double maxOutput=0; 
	private double minOutput=0;

	private double setpoint=0;

	private double lastActual=0;

	private boolean firstRun=true;
	private boolean reversed=false;

	private double outputRampRate=0;
	private double lastOutput=0;

	private double outputFilter=0;

	private double setpointRange=0;

	private double continuousMax=0;
	private double continuousMin=0;

	/** User-defined function for precise modeling of the controlled system
	 * Note, setpoint and error are constrained by any configuration options provided by the system.
	 */
	public interface FeedForwardLambda{
		public double compute(double setpoint, double actual, double error);
	}
	private FeedForwardLambda feedForwardLambda = (s,a,e)->{return 0.0;};


	//**********************************
	// Constructor functions
	//**********************************
	
	/**
	 * Create a MiniPID class object. 
	 * See setP, setI, setD methods for more detailed parameters.
	 * @param p Proportional gain. Large if large difference between setpoint and target. 
	 * @param i Integral gain.  Becomes large if setpoint cannot reach target quickly. 
	 * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents P and I terms from causing overshoot.
	 */
	public MiniPID(double p, double i, double d){
		P=p; I=i; D=d;
		checkSigns();
		}

	/**
	 * Create a MiniPID class object. 
	 * See setP, setI, setD, setF methods for more detailed parameters.
	 * @param p Proportional gain. Large if large difference between setpoint and target. 
	 * @param i Integral gain.  Becomes large if setpoint cannot reach target quickly. 
	 * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents P and I terms from causing overshoot.
	 * @param f Feed-forward gain. Open loop "best guess" for the output should be. Only useful if setpoint represents a rate.
	 */
	public MiniPID(double p, double i, double d, double f){
		P=p; I=i; D=d; F=f;
		checkSigns();
		}

	//**********************************
	// Configuration functions
	//**********************************
	/**
	 * Configure the Proportional gain parameter. <br>
	 * This responds quickly to changes in setpoint, and provides most of the initial driving force
	 * to make corrections. <br>
	 * Some systems can be used with only a P gain, and many can be operated with only PI.<br>
	 * For position based controllers, this is the first parameter to tune, with I second. <br>
	 * For rate controlled systems, this is often the second after F.
	 *  
	 * @param p Proportional gain. Affects output according to <b>output+=P*(setpoint-current_value)</b>
	 */
	public MiniPID setP(double p){
		P=p;
		checkSigns();
		return this;
	}

	/**
	 * Changes the I parameter <br>
	 * This is used for overcoming disturbances, and ensuring that the controller always gets to the control mode. 
	 * Typically tuned second for "Position" based modes, and third for "Rate" or continuous based modes. <br>
	 * Affects output through <b>output+=previous_errors*Igain ;previous_errors+=current_error</b>
	 * 
	 * @see {@link #setMaxIOutput(double) setMaxIOutput} for how to restrict
	 *
	 * @param i New gain value for the Integral term
	 */
	public MiniPID setI(double i){
		if(I!=0){
			errorSum=errorSum*I/i;
			}
		if(maxIOutput!=0){
			maxError=maxIOutput/i;
		}
		I=i;
		checkSigns();
		 // Implementation note: 
		 // This Scales the accumulated error to avoid output errors. 
		 // As an example doubling the I term cuts the accumulated error in half, which results in the 
		 // output change due to the I term constant during the transition.
		 return this;
	}

	/**
	 * Changes the D parameter <br>
	 * This has two primary effects:
	 * <list>
	 * <li> Adds a "startup kick" and speeds up system response during setpoint changes
	 * <li> Adds "drag" and slows the system when moving toward the target
	 * </list>
	 * A small D value can be useful for both improving response times, and preventing overshoot.
	 * However, in many systems a large D value will cause significant instability, particularly 
	 * for large setpoint changes.
	 * <br>
	 * Affects output through <b>output += -D*(current_input_value - last_input_value)</b>
	 *
	 * @param d New gain value for the Derivative term
	 */
	public MiniPID setD(double d){
		D=d;
		checkSigns();
		return this;
	}

	/**
	 * Configure the FeedForward parameter. <br>
	 * This is excellent for velocity, rate, and other  continuous control modes where you can 
	 * expect a rough output value based solely on the setpoint.<br>
	 * Should not be used in "position" based control modes.<br>
	 * Affects output according to <b>output+=F*Setpoint</b>. Note, that a F-only system is actually open loop.
	 * 
	 * @param f Feed forward gain. 
	 */
	public MiniPID setF(double f){
		F=f;
		checkSigns();
		return this;
	}

	/**
	 * Configure the FeedForward parameter using arbitrary function and system parameters <br>
	 * This allows for precise modelling of the expected beaviour of your system,
	 * allowing effective PID use on a wider variety of systems.
	 * <br>
	 * Examples:
	 * <li> For a position system that lifts elevator, the lambda would return a constant value, which 
	 * represents the motor power necessary to counteract gravity. 
	 * <li> For a positional system controlling a vertically rotating arm, kPowerAtFullExtension*sin(angle) operation
	 * to counteract gravity
	 * @param ff lambda that models the system based on known parameters 
	 * @return
	 */
	public MiniPID setF(FeedForwardLambda ff){
		this.feedForwardLambda = ff;
		return this;
	}

	/** 
	 * Configure the PID object.
	 * See setP, setI, setD methods for more detailed parameters.
	 * @param p Proportional gain. Large if large difference between setpoint and target. 
	 * @param i Integral gain.  Becomes large if setpoint cannot reach target quickly. 
	 * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents P and I terms from causing overshoot.
	 */
	public MiniPID setPID(double p, double i, double d){
		P=p;D=d;
		//Note: the I term has additional calculations, so we need to use it's 
		//specific method for setting it.
		setI(i);
		checkSigns();
		return this;
	}

	/** 
	 * Configure the PID object.
	 * See setP, setI, setD, setF methods for more detailed parameters.
	 * @param p Proportional gain. Large if large difference between setpoint and target. 
	 * @param i Integral gain.  Becomes large if setpoint cannot reach target quickly. 
	 * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents P and I terms from causing overshoot.
	 * @param f Feed-forward gain. Open loop "best guess" for the output should be. Only useful if setpoint represents a rate.
	 */
	public MiniPID setPID(double p, double i, double d,double f){
		P=p;D=d;F=f;
		//Note: the I term has additional calculations, so we need to use it's 
		//specific method for setting it.
		setI(i);
		checkSigns();
		return this;
	}

	/**
	 * Set the maximum output value contributed by the I component of the system
	 * This can be used to prevent large windup issues and make tuning simpler
	 * @param maximum. Units are the same as the expected output value
	 */
	public MiniPID setMaxIOutput(double maximum){
		// Internally maxError and Izone are similar, but scaled for different purposes. 
		// The maxError is generated for simplifying math, since calculations against 
		// the max error are far more common than changing the I term or Izone. 
		maxIOutput=maximum;
		if(I!=0){
			maxError=maxIOutput/I;
		}
		return this;
	}

	/**
	 * Specify a maximum output range. <br>
	 * When one input is specified, output range is configured to 
	 * <b>[-output, output]</b>
	 * @param output
	 */
	public MiniPID setOutputLimits(double output){
		setOutputLimits(-output,output);
		return this;
	}

	/**
	 * Specify a  maximum output.
	 * When two inputs specified, output range is configured to 
	 * <b>[minimum, maximum]</b>
	 * @param minimum possible output value
	 * @param maximum possible output value
	 */
	public MiniPID setOutputLimits(double minimum,double maximum){
		if(maximum<minimum) return this;
		maxOutput=maximum;
		minOutput=minimum;

		// Ensure the bounds of the I term are within the bounds of the allowable output swing
		if(maxIOutput==0 || maxIOutput>(maximum-minimum) ){
			setMaxIOutput(maximum-minimum);
		}
		return this;
	}

	/** 
	 * Set the operating direction of the PID controller
	 * @param reversed Set true to reverse PID output
	 */
	public MiniPID setDirection(boolean reversed){
		this.reversed=reversed;
		return this;
	}

	//**********************************
	// Primary operating functions
	//**********************************

	/**
	 * Configure setpoint for the PID calculations<br>
	 * This represents the target for the PID system's, such as a 
	 * position, velocity, or angle. <br>
	 * @see MiniPID#getOutput(actual) <br>
	 * @param setpoint
	 */
	public MiniPID setSetpoint(double setpoint){
		this.setpoint=setpoint;
		return this;
	}

	/**
	 * Calculate the output value for the current PID cycle.<br>
	 * @param actual The monitored value, typically as a sensor input.
	 * @param setpoint The target value for the system
	 * @return calculated output value for driving the system
	 */
	public double getOutput(double actual, double setpoint){
		double output;
		double Poutput;
		double Ioutput;
		double Doutput;
		double Foutput;
		double continuousHalfRange;

		this.setpoint=setpoint;

		// Do the simple parts of the calculations
		double error=setpoint-actual;

		// If we're in continous mode, wrap our error to better match the system
		if(continuousMin != continuousMax){
			continuousHalfRange = (continuousMax-continuousMin)/2.0;
			error %= (continuousHalfRange*2);
			if(error>continuousHalfRange) error-=2*continuousHalfRange;
			if(error<-continuousHalfRange) error+=2*continuousHalfRange;
		}
		
		// Apply the setpoint range, which effectively constrains our error within a smaller range
		if (setpointRange != 0) {
			error = constrain(error,-setpointRange,setpointRange);
		}

		// Calculate F output. Notice, this depends only on the setpoint, and not the error. 
		Foutput=F*setpoint;
		// Handle user-provided functions to model things we can't handle well 
		Foutput += feedForwardLambda.compute(setpoint, actual, error);

		// Calculate P term
		Poutput=P*error;   

		// If this is our first time running this, we don't actually _have_ a previous input or output. 
		// For sensor, sanely assume it was exactly where it is now.
		// For last output, we can assume it's the current time-independent outputs. 
		if(firstRun){
			lastActual=actual;
			lastOutput=Poutput+Foutput;
			firstRun=false;
		}

		// Calculate D Term
		// Note, this is negative. This actually "slows" the system if it's doing
		// the correct thing, and small values helps prevent output spikes and overshoot 
		Doutput= -D*(actual-lastActual);
		lastActual=actual;

		// The Iterm is more complex. There's several things to factor in to make it easier to deal with.
		// 1. maxIoutput restricts the amount of output contributed by the Iterm.
		// 2. prevent windup by not increasing errorSum if we're already running against our max Ioutput
		// 3. prevent windup by not increasing errorSum if output is output=maxOutput    
		Ioutput=I*errorSum;
		if(maxIOutput!=0){
			Ioutput=constrain(Ioutput,-maxIOutput,maxIOutput); 
		}    

		// And, finally, we can just add the terms up
		output=Foutput + Poutput + Ioutput + Doutput;

		// Figure out what we're doing with the error.
		if(minOutput!=maxOutput && !bounded(output, minOutput,maxOutput) ){
			errorSum=error; 
			// reset the error sum to a sane level
			// Setting to current error ensures a smooth transition when the P term 
			// decreases enough for the I term to start acting upon the controller
			// From that point the I term will build up as would be expected
		}
		else if(outputRampRate!=0 && !bounded(output, lastOutput-outputRampRate,lastOutput+outputRampRate) ){
			errorSum=error; 
		}
		else if(maxIOutput!=0){
			errorSum=constrain(errorSum+error,-maxError,maxError);
			// In addition to output limiting directly, we also want to prevent I term 
			// buildup, so restrict the error directly
		}
		else{
			errorSum+=error;
		}

		// Restrict output to our specified output and ramp limits
		if(outputRampRate!=0){
			output=constrain(output, lastOutput-outputRampRate,lastOutput+outputRampRate);
		}
		if(minOutput!=maxOutput){ 
			output=constrain(output, minOutput,maxOutput);
			}
		if(outputFilter!=0){
			output=lastOutput*outputFilter+output*(1-outputFilter);
		}

		// Get a test printline with lots of details about the internal 
		// calculations. This can be useful for debugging. 
		// System.out.printf("Final output %5.2f [ %5.2f, %5.2f , %5.2f  ], eSum %.2f\n",output,Poutput, Ioutput, Doutput,errorSum );
		// System.out.printf("%5.2f\t%5.2f\t%5.2f\t%5.2f\n",output,Poutput, Ioutput, Doutput );

		lastOutput=output;
		return output;
	}

	/**
	 * Calculate the output value for the current PID cycle.<br>
	 * In no-parameter mode, this uses the last sensor value, 
	 * and last setpoint value. <br>
	 * Not typically useful, and use of parameter modes is suggested. <br>
	 * @return calculated output value for driving the system
	 */
	public double getOutput(){
		return getOutput(lastActual,setpoint);
	}

	/**
	 * Calculate the output value for the current PID cycle.<br>
	 * In one parameter mode, the last configured setpoint will be used.<br>
	 * @see MiniPID#setSetpoint()
	 * @param actual The monitored value, typically as a sensor input.
	 * @param setpoint The target value for the system
	 * @return calculated output value for driving the system
	 */
	public double getOutput(double actual){
		return getOutput(actual,setpoint);
	}

	/**
	 * Resets the controller. This erases the I term buildup, and removes 
	 * D gain on the next loop.<br>
	 * This should be used any time the PID is disabled or inactive for extended
	 * duration, and the controlled portion of the system may have changed due to
	 * external forces.
	 */
	public MiniPID reset(){
		firstRun=true;
		errorSum=0;
		return this;
	}

	/**
     * Set the maximum rate the output can increase per cycle.<br>
     * This can prevent sharp jumps in output when changing setpoints or 
     * enabling a PID system, which might cause stress on physical or electrical
     * systems.  <br>
     * Can be very useful for fast-reacting control loops, such as ones 
     * with large P or D values and feed-forward systems.
     * 
	 * @param rate, with units being the same as the output
	 */
	public MiniPID setOutputRampRate(double rate){
		outputRampRate=rate;
		return this;
	}

	/** 
     * Set a limit on how far the setpoint can be from the current position
	 * <br>Can simplify tuning by helping tuning over a small range applies to a much larger range. 
	 * <br>This limits the reactivity of P term, and restricts impact of large D term
	 * during large setpoint adjustments. Increases lag and I term if range is too small.
	 * @param range, with units being the same as the expected sensor range. 
	 */
	public MiniPID setSetpointRange(double range){
		setpointRange=range;
		return this;
	}
	
	/**
	 * Tell the controller that min and max represent the same same physical value.
	 * <br>
	 * This is notably useful for position control of angular values. In many such systems, 
	 * 0 and 360 represent the same heading, but without special handling you'll often
	 * correct the error going in an inefficient direction.
	 * <br>
	 * Note, that restricting setpoints or sensor inputs to the specified range are not
	 * required. The controller wraps all values back within this range automatically.
	 * 
	 * @param minSensorValue Lower bound of the continuous range
	 * @param maxSensorValue Lower bound of the continuous range
	 */
	public MiniPID setContinuousMode(double minSensorValue, double maxSensorValue){
		this.continuousMin = minSensorValue;
		this.continuousMax = maxSensorValue;
		return this;
	}

	/**
	 * Disable the continous mode, and resume normal operation.
	 *  
	 */
	public MiniPID setContinousModeOff(){
		this.continuousMin=0;
		this.continuousMax=0;
		return this;
	}

	/**
     * Set a filter on the output to reduce sharp oscillations. <br>
	 * 0.1 is likely a sane starting value. Larger values use historical data
	 * more heavily, with low values weigh newer data. 0 will disable, filtering, and use 
	 * only the most recent value. <br>
	 * Increasing the filter strength will P and D oscillations, but force larger I 
	 * values and increase I term overshoot.<br>
	 * Uses an exponential wieghted rolling sum filter, according to a simple <br>
	 * <pre>output*(1-strength)*sum(0..n){output*strength^n}</pre> algorithm.
	 * @param output valid between [0..1), meaning [current output only.. historical output only)
	 */
	public MiniPID setOutputFilter(double strength){
		if(strength==0 || bounded(strength,0,1)){
		outputFilter=strength;
		}
		return this;
	}


	//**************************************
	// Helper functions
	//**************************************

	/**
	 * Forces a value into a specific range
	 * @param value input value
	 * @param min maximum returned value
	 * @param max minimum value in range
	 * @return Value if it's within provided range, min or max otherwise 
	 */
	private double constrain(double value, double min, double max){
		if(value > max){ return max;}
		if(value < min){ return min;}
		return value;
	}  

	/**
	 * Test if the value is within the min and max, inclusive
	 * @param value to test
	 * @param min Minimum value of range
	 * @param max Maximum value of range
	 * @return true if value is within range, false otherwise
	 */
	private boolean bounded(double value, double min, double max){
		// Note, this is an inclusive range. This is so tests like
		// `bounded(constrain(0,0,1),0,1)` will return false.
		// This is more helpful for determining edge-case behaviour
		// than <= is.
		return (min<value) && (value<max);
	}

	/**
	 * To operate correctly, all PID parameters require the same sign
	 * This should align with the {@literal}reversed value
	 */
	private void checkSigns(){
		if(reversed){  // all values should be below zero
			if(P>0) P*=-1;
			if(I>0) I*=-1;
			if(D>0) D*=-1;
			if(F>0) F*=-1;
		}
		else{  // all values should be above zero
			if(P<0) P*=-1;
			if(I<0) I*=-1;
			if(D<0) D*=-1;
			if(F<0) F*=-1;
		}
	}
}