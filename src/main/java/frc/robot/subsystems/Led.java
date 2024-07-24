package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.constants.Ports;

public class Led implements ISubsystem{

    private AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue;
    private PWMSparkMax led;
    public boolean ledRunning;

    int[] array;

    private Led(PWMSparkMax led){
      ledRunning = false;

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

    @Override
    public void onLoop(){}


    public void    red() { led.set(0.61); }
    public void orange() { led.set(0.65); }
    public void yellow() { led.set(0.69); }
    public void  green() { led.set(0.77); }
    public void   blue() { led.set(0.87); }
    public void violet() { led.set(0.91); }
    public void  white() { led.set(0.93); }
    public void  black() { led.set(0.99); }

    //all color values found here: https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf

    @Override
    public void submitTelemetry(){}

    @Override
    public void receiveOptions(){}
  }