package frc.robot;

public final class DeadBand {
    // deadband maybe 0.2, response maybe 2.5
  // -1.0 <= input <= 1.0
  public double deadbandresponse(double input, double deadband, double response) 
  {
    double sign = 1.0;
    double value = 0.0;
    double deadbandnorm = 1.0;

    // sanity check the arguments, so no math errors
    if((deadband > 0.0) && (deadband < 0.5))
    {
      deadbandnorm = 1.0/(1.0-deadband);
    }
    else
    {
      deadband = 0.0;
    }
    if(input >  1.0){ input =  1.0;}
    if(input < -1.0){ input = -1.0;}
    if(response < 1.0){response = 1.0;}
    if(response > 5.0){response = 5.0;}

    if(input < 0.0)
    {
      sign = -1.0;
      input = -input;
    }

    // compute result
    value = (input - deadband);
    if (value < 0.0) value = 0.0;
    value = value*deadbandnorm;
    value = Math.pow(value, response);
    if(value > 1.0){value = 1.0;}
    return value*sign;
  }
}
