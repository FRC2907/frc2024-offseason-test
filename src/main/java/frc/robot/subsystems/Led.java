package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Control;
import frc.robot.constants.Ports;

public class Led implements ISubsystem{

    private AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue;
    private PWMSparkMax led;

    private int[] array;

    private Led(PWMSparkMax led){
      array = new int[m_ledBuffer.getLength()];
      for (int i = 0; i < m_ledBuffer.getLength(); i++){
        array[i] = i;
      }
    }

    private static Led instance;

    public static Led getInstance(){
      if (instance == null){
        PWMSparkMax led = new PWMSparkMax(Ports.PWM.LED);
        instance = new Led(led);
      }
      return instance;
    }
  

    public void rainbow(){
      for (int i = 0; i < m_ledBuffer.getLength(); i++){
        final int hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
        m_ledBuffer.setHSV(i, hue, 255, 128);
      }
      m_rainbowFirstPixelHue += 3;
      m_rainbowFirstPixelHue %= 180;
    }

    public void lionPride(){
      for (int i = 0; i < m_ledBuffer.getLength(); i++){
        if (array[i] < m_ledBuffer.getLength() / 2){
          m_ledBuffer.setHSV(i, 213, 99, 77); //TODO s and v are entered for percentages, change if necessary
        } else if (array[i] > m_ledBuffer.getLength() / 2){
          m_ledBuffer.setHSV(i, 33, 100, 100); //TODO s and v are entered for percentages, change if necessary
        }
      }
      for (int i = 0; i < m_ledBuffer.getLength(); i++){
        array[i] += 1;
        array[i] %= m_ledBuffer.getLength();
      }
    }



    public String getColor(){
      if (led.get() == Control.led.red){
        return "red";
      }
      if (led.get() == Control.led.orange){
        return "orange";
      }
      if (led.get() == Control.led.yellow){
        return "yellow";
      }
      if (led.get() == Control.led.green){
        return "green";
      }
      if (led.get() == Control.led.blue){
        return "blue";
      }
      if (led.get() == Control.led.violet){
        return "violet";
      }
      if (led.get() == Control.led.white){
        return "white";
      }
      if (led.get() == Control.led.black){
        return "black";
      }
      return null;
    }

    

    @Override
    public void onLoop(){}


    public void    red() { led.set(Control.led.red); }
    public void orange() { led.set(Control.led.orange); }
    public void yellow() { led.set(Control.led.yellow); }
    public void  green() { led.set(Control.led.green); }
    public void   blue() { led.set(Control.led.blue); }
    public void violet() { led.set(Control.led.violet); }
    public void  white() { led.set(Control.led.white); }
    public void  black() { led.set(Control.led.black); }



    @Override
    public void submitTelemetry(){
      SmartDashboard.putString("led/color", this.getColor());
    }

    @Override
    public void receiveOptions(){}
  }