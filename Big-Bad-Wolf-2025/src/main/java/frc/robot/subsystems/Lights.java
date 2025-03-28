// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;

// Creates a new Lights subsystem.
public class Lights extends SubsystemBase 
{
  private final Timer m_LightTimer = new Timer();
  private final Timer m_BootUpTimer = new Timer();
  private final Timer m_RicochetTimer = new Timer();
  AddressableLEDBuffer m_ledBuffer1 = new AddressableLEDBuffer(46);

  private final AddressableLEDBufferView m_leftData = m_ledBuffer1.createView(0, 23);
  private final AddressableLEDBufferView m_rightData = m_ledBuffer1.createView(24, 45).reversed();

  private final AddressableLEDBufferView m_centerData = m_ledBuffer1.createView(8, 14);
  private final AddressableLEDBufferView m_centerBackData = m_ledBuffer1.createView(32, 38);

  AddressableLED m_led = new AddressableLED(0);
  private int m_BrightnessCount;
  private boolean m_IsGoingUp = true;

  private LIGHT_CODES m_CurrentCode;

  /**
   * Lights subsystem constructor.
   */
  public Lights() 
  {
    m_LightTimer.start();
    m_BootUpTimer.start();
    m_RicochetTimer.start();
    m_led.setLength(m_ledBuffer1.getLength());

    this.LightChooser(LIGHT_CODES.FLASHING_RED);
    m_CurrentCode = LIGHT_CODES.FADING_ORANGE_AND_BLUE;
  }

  @Override
  public void periodic() 
  {
    if(m_LightTimer.get() > 0.5)
    {
      m_LightTimer.reset();
    }

    if(m_RicochetTimer.get() > 2)
    {
      m_RicochetTimer.reset();
    }

    this.LightChooser(m_CurrentCode);
  }


  /**
   * Different light settings that can be called.
   */
  public static enum LIGHT_CODES
  {
    SOLID_GREEN,
    FLASHING_GREEN,
    SOLID_BLUE,
    FLASHING_BLUE,
    SOLID_ORANGE,
    FLASHING_ORANGE,
    SOLID_RED,
    FLASHING_RED,
    FLASHING_ORANGE_AND_BLUE,
    FLASH_ALTERNATE_ORANGE_AND_BLUE,
    FADING_BLUE,
    FADING_ORANGE,
    FADING_ORANGE_AND_BLUE,
    BOOT_UP_PATTERN,
    CENTER_INTAKE_FLASH;
  }

  public void SetLightCode(LIGHT_CODES code)
  {
    this.m_CurrentCode = code;
  }

  /**
   * @brief Chooses which light setting from LIGHT_CODES to utilize.
   * @param type The light setting the lights are set to.
   */
  public void LightChooser(LIGHT_CODES type)
  {
    switch(type)
    {
      case SOLID_GREEN:
        this.setSolidColor(60, 255, 255);
      break;

      case FLASHING_GREEN:
        this.setColorFlash(60, 255, 255);
      break;

      case SOLID_BLUE:
        this.setSolidColor(100, 200, 100);
      break;

      case FLASHING_BLUE:
        this.setColorFlash(100, 200, 100);
      break;

      case SOLID_ORANGE:
        this.setSolidColor(4, 255, 255);
      break;

      case FLASHING_ORANGE:
        this.setColorFlash(4, 255, 255);
      break;

      case SOLID_RED:
        this.setSolidColor(1, 255, 255);
      break;

      case FLASHING_RED:
        this.setColorFlash(1, 255, 255);
      break;

      case FLASHING_ORANGE_AND_BLUE:
        this.setColorFlashSwap(4, 100, 255, 255);
      break;

      case FLASH_ALTERNATE_ORANGE_AND_BLUE:
        this.setHalfAndHalf(4, 100, 255, 255);
      break;

      case FADING_BLUE:
        this.fadeInFadeOut(100, 200);
      break;

      case FADING_ORANGE:
        this.fadeInFadeOut(4, 255);
      break;

      case BOOT_UP_PATTERN:
        // if(m_BootUpTimer.hasElapsed(2))
        // {
        //     this.setHalfAndHalf(4, 100, 255, 255);
        // }
        this.setLEDRicochet();
        // if(m_BootUpTimer.advanceIfElapsed(10) == true)
        // {
        //     this.setLEDRicochet();
        // }

        // // USE THIS ONE!!!!!!!!
        // if(m_BootUpTimer.get() < 4)
        // {
        //   this.setColorFlashSwap(4, 100, 255, 255);
        // }
        // else
        // {
        //   m_BootUpTimer.stop();
        // }

      break;

      case CENTER_INTAKE_FLASH:
        this.setCenterIntakeFlash(4, 0, 255);
      break;

      default:
        this.setOffLights();
      break;
    }
  }

  /**
   * @brief Sets the lights to a specified color.
   * @param hue The hue (color) of the lights.
   * @param saturation The saturation (amount of gray) in the lights.
   * @param brightness The brightness of the lights.
   */
  public void setSolidColor(int hue, int saturation, int brightness)
  {
    for(int i = 0; i < m_ledBuffer1.getLength(); i++) 
    {
      m_ledBuffer1.setHSV(i, hue, saturation, brightness);
    }
    m_led.setData(m_ledBuffer1);
    m_led.start();
  }

  /**
   * @brief Sets the center three LEDs of the LED strip on both halves to a specified flashing color to signal intaking.
   * @param hue The hue(color) of the lights.
   * @param saturation The saturation(amount of gray) in the lights.
   * @param brightness The brightness of the lights.
   */
  public void setCenterIntakeFlash(int hue, int saturation, int brightness)
  {
    if(m_LightTimer.get() > .25)
    {
        for(int i = 0; i < m_centerData.getLength(); i++)
        {
            m_centerData.setHSV(i, hue, saturation, brightness);
        }
        m_led.setData(m_ledBuffer1);

        for(int i = 0; i < m_centerBackData.getLength(); i++)
        {
            m_centerBackData.setHSV(i, hue, saturation, brightness);
        }
    }
    else
    {
        setOffLights();
        m_led.setData(m_ledBuffer1);
    }
    m_led.start();
  }

  /**
   * @brief Sets the lights to continuously flash a specified color.
   * @param hue The hue (color) of the lights.
   * @param saturation The saturation (amount of gray) in the lights.
   * @param brightness The brightness of the lights.
   */    
  public void setColorFlash(int hue, int saturation, int brightness)
  {
    if(m_LightTimer.get() > .25)
      {
        setOffLights();
        m_led.setData(m_ledBuffer1);
      }
    else
      {
        for(int x = 0; x < m_ledBuffer1.getLength(); x++)
        {
          m_ledBuffer1.setHSV(x, hue, saturation, brightness);
        }
        m_led.setData(m_ledBuffer1);
      }
    m_led.start();
  }

  /**
   * @brief Sets a fading in and out brightness pattern for a specified hue and saturation. 
   * @param hue The hue (color) of the lights.
   * @param saturation The saturation (amount of gray) in the lights.
   */
  public void fadeInFadeOut(int hue, int saturation)
  {
    if(m_LightTimer.get() > 0.25)
    {
      if(m_IsGoingUp)
      {
        m_BrightnessCount += 6;
        if(m_BrightnessCount > 244)
        {
          m_IsGoingUp = false;
        }
      }
      else
      {
        m_BrightnessCount -= 6;
        if(m_BrightnessCount < 42)
        {
          m_IsGoingUp = true;
        }
      }
      for(int x = 0; x < m_ledBuffer1.getLength(); x++)
      {
        m_ledBuffer1.setHSV(x, hue, saturation, m_BrightnessCount);
      }
      m_led.setData(m_ledBuffer1);
    }
    m_led.start();
  }

  /**
   * @brief Splits the light strand into two colors that continously flash and alternate halves.
   * @param hue The hue(color) of the first half the lights.
   * @param hue2 The hue(color) of the second half of the lights.
   * @param saturation The saturation (amount of gray) in the lights.
   * @param brightness The brightness of the lights.
   */
  public void setHalfAndHalf(int hue, int hue2, int saturation, int brightness)
  {
    if(m_LightTimer.get() > .25)
    {
      for(int i = 0; i < m_ledBuffer1.getLength() / 2; i++)
      {
        m_ledBuffer1.setHSV(i, hue, saturation, brightness);
      }
      m_led.setData(m_ledBuffer1);
      for(int i = m_ledBuffer1.getLength() /2; i < m_ledBuffer1.getLength(); i++)
      {
        m_ledBuffer1.setHSV(i, hue2, saturation, brightness);
      }
      m_led.setData(m_ledBuffer1);
    }
    else
    {
      for(int i = 0; i < m_ledBuffer1.getLength() / 2; i++)
      {
        m_ledBuffer1.setHSV(i, hue2, saturation, brightness);
      }
      m_led.setData(m_ledBuffer1);
      for(int i = m_ledBuffer1.getLength() /2; i < m_ledBuffer1.getLength(); i++)
      {
        m_ledBuffer1.setHSV(i, hue, saturation, brightness);
      }
      m_led.setData(m_ledBuffer1);
      
    }
    m_led.start();
  }

  /**
   * @brief Sets the entire color strip to one color and flashes a different color every 0.25 seconds.
   * @param hue The hue(color) of the first color flashing.
   * @param hue2 The hue(color) of the second color flashing.
   * @param saturation The saturation(amount of gray) in the lights.
   * @param brightness The brightness of the lights.
   */
  public void setColorFlashSwap(int hue, int hue2, int saturation, int brightness)
  {
    if(m_LightTimer.get() > .25)
      {
        for(int x = 0; x < m_ledBuffer1.getLength(); x++)
        {
          m_ledBuffer1.setHSV(x, hue, saturation, brightness);
        }
        m_led.setData(m_ledBuffer1);
      }
    else
      {
        for(int x = 0; x < m_ledBuffer1.getLength(); x++)
        {
          m_ledBuffer1.setHSV(x, hue, saturation, brightness);
        }
        m_led.setData(m_ledBuffer1);
      }
    m_led.start();
  }

  /**
   * @brief Sets a blue section to ricochet parallaled across both sides of the LED strip.
   * @param hue The hue(color) of the lights.
   * @param saturation The saturation(amount of gray) in the lights.
   * @param brightness The brightness of the lights.
   */
  public void setLEDRicochet()
  {
    // Make the section of blankness bigger (Map.of(0.8, Color.kWhite);)
    Map<Double, Color> maskSteps = Map.of(0.96, Color.kWhite);
    // LEDPattern base = LEDPattern.rainbow(255, 255);
    Distance LED_SPACING = Meters.of(1.0 / 60);
    LEDPattern base = LEDPattern.solid(Color.kCornflowerBlue);
    LEDPattern mask =
        LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(45));

    LEDPattern maskBackward =
    LEDPattern.steps(maskSteps).scrollAtAbsoluteSpeed(InchesPerSecond.of(-20), LED_SPACING);

    LEDPattern pattern = base.mask(mask);
    LEDPattern backwardsPattern = base.mask(maskBackward);

    // Apply the LED pattern to the data buffer
    // pattern.applyTo(m_ledBuffer1);
    if(m_RicochetTimer.get() > 1)
    {
      pattern.applyTo(m_leftData);
      pattern.applyTo(m_rightData);
    }
    else
    {
      backwardsPattern.applyTo(m_leftData);
      backwardsPattern.applyTo(m_rightData);
    }

    // Write the data to the LED strip
    m_led.setData(m_ledBuffer1);
    m_led.start();
  }

  /**
   * @brief Turns off all of the lights when called.
   */
  public void setOffLights()
  {
    for (int x = 0; x < m_ledBuffer1.getLength(); x++)
    {
      m_ledBuffer1.setHSV(x, 0, 0, 0);
    }
  }
}