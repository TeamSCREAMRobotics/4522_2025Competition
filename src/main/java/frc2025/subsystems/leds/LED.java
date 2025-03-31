// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025.subsystems.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import math.ScreamMath;

/** Add your docs here. */
public class LED extends SubsystemBase {
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final int length = LEDConstants.STRIP_LENGTH;

  public LED() {
    leds = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(LEDConstants.STRIP_LENGTH);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();
  }

  // From FRC 6328 Mechanical Advantage
  // https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/subsystems/leds/Leds.java

  public void solid(Color color) {
    if (color != null) {
      for (int i = 0; i < length; i++) {
        buffer.setLED(i, color);
      }
    }
  }

  public void solid(double percent, Color color) {
    for (int i = 0; i < MathUtil.clamp(length * percent, 0, length); i++) {
      buffer.setLED(i, color);
    }
  }

  public void strobe(Color color, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(on ? color : Color.kBlack);
  }

  public void breathe(Color c1, Color c2, double duration) {
    breathe(c1, c2, duration, Timer.getFPGATimestamp());
  }

  public void breathe(Color c1, Color c2, double duration, double timestamp) {
    double x =
        ((timestamp % LEDConstants.BREATHE_DURATION) / LEDConstants.BREATHE_DURATION)
            * 2.0
            * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(new Color(red, green, blue));
  }

  public void rainbow(double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < length; i++) {
      x += xDiffPerLed;
      x %= 180.0;
      if (i >= 0) {
        buffer.setHSV(i, (int) x, 255, 255);
      }
    }
  }

  public Color rainbow(double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    return Color.fromHSV((int) x, 255, 255);
  }

  public void wave(Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < length; i++) {
      x += xDiffPerLed;
      if (i >= 0) {
        double ratio = (Math.pow(Math.sin(x), LEDConstants.BREATHE_DURATION) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) {
          ratio = (-Math.pow(Math.sin(x + Math.PI), LEDConstants.BREATHE_DURATION) + 1.0) / 2.0;
        }
        if (Double.isNaN(ratio)) {
          ratio = 0.5;
        }
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        buffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  public void stripes(List<Color> colors, int length, double duration) {
    int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
    for (int i = 0; i < length; i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }

  public void scaledTarget(Color color, double currentValue, double targetValue) {
    if (targetValue == 0) return;

    int mapped = (int) Math.round(ScreamMath.mapRange(currentValue, 0, targetValue, 0, length));
    mapped = MathUtil.clamp(mapped, 0, length);

    for (int i = 0; i < mapped; i++) {
      buffer.setLED(i, color);
    }

    for (int i = mapped; i < length; i++) {
      buffer.setLED(i, Color.kBlack);
    }
  }

  public void centerScaledTarget(Color color, double currentValue, double targetValue) {
    if (targetValue == 0) return;

    int mapped =
        (int) Math.round(ScreamMath.mapRange(currentValue, 1.0, targetValue, 0, length / 2));
    mapped = MathUtil.clamp(mapped, 0, length / 2);

    boolean isOdd = (length % 2) == 1;

    for (int i = 0; i < mapped + (isOdd ? 1 : 0); i++) {
      buffer.setLED(i, color);
    }

    for (int i = length - 1; i >= length - mapped; i--) {
      buffer.setLED(i, color);
    }

    for (int i = mapped; i < length - mapped; i++) {
      buffer.setLED(i, Color.kBlack);
    }
  }

  public void larson(Color color, double duration) {
    int trailLength = 3;
    for (int i = 0; i < length; i++) {
      buffer.setRGB(i, 0, 0, 0);
    }

    double currentTime = Timer.getFPGATimestamp();
    double cycleTime = currentTime % duration;
    double position;

    if (cycleTime < duration / 2) {
      position = (cycleTime / (duration / 2)) * (length - 1);
    } else {
      position = ((duration - cycleTime) / (duration / 2)) * (length - 1);
    }

    int trailPixels = (int) (trailLength);

    int mainPos = (int) Math.round(position);
    if (mainPos >= 0 && mainPos < length) {
      buffer.setLED(mainPos, color);

      for (int i = 1; i <= trailPixels; i++) {
        double factor = 0.7 - ((double) i / (trailPixels + 1));

        Color trailColor = new Color(color.red * factor, color.green * factor, color.blue * factor);

        if (mainPos - i >= 0) {
          buffer.setLED(mainPos - i, trailColor);
        }

        if (mainPos + i < length) {
          buffer.setLED(mainPos + i, trailColor);
        }
      }
    }
  }

  public void mapped(double currentValue) {
    int mapped = (int) ScreamMath.mapRange(currentValue, 0, 100, 0, 180);
    mapped = MathUtil.clamp(mapped, 0, 180);
    solid(Color.fromHSV(mapped, 255, 255));
  }

  public Color mappedColor(double value) {
    int mapped = (int) ScreamMath.mapRange(value, 0, 100, 0, 180);
    mapped = MathUtil.clamp(mapped, 0, 180);
    return Color.fromHSV(mapped, 255, 255);
  }

  @Override
  public void periodic() {
    leds.setData(buffer);
    // System.out.println("test");
  }

  public Command solidCommand(Color color) {
    return run(() -> solid(color)).ignoringDisable(true);
  }

  public Command solidCommand(Color color, double percent) {
    return run(() -> solid(percent, color)).ignoringDisable(true);
  }

  public Command strobeCommand(Color color, double duration) {
    return run(() -> strobe(color, duration)).ignoringDisable(true);
  }

  public Command strobeCommand(Supplier<Color> color, double duration) {
    return run(() -> strobe(color.get(), duration)).ignoringDisable(true);
  }

  public Command breatheCommand(Color color1, Color color2, double duration) {
    return run(() -> breathe(color1, color2, duration)).ignoringDisable(true);
  }

  public Command rainbowCommand(double cycleLength, double duration) {
    return run(() -> rainbow(cycleLength, duration)).ignoringDisable(true);
  }

  public Command waveCommand(Color color1, Color color2, double cycleLength, double duration) {
    return run(() -> wave(color1, color2, cycleLength, duration)).ignoringDisable(true);
  }

  public Command waveCommand(
      Supplier<Color> color1, Supplier<Color> color2, double cycleLength, double duration) {
    return run(() -> wave(color1.get(), color2.get(), cycleLength, duration)).ignoringDisable(true);
  }

  public Command waveCommand(
      Supplier<Color> color1, Supplier<Color> color2, double cycleLength, DoubleSupplier duration) {
    return run(() -> wave(color1.get(), color2.get(), cycleLength, duration.getAsDouble()))
        .ignoringDisable(true);
  }

  public Command stripeCommand(List<Color> colors, int length, double duration) {
    return run(() -> stripes(colors, length, duration)).ignoringDisable(true);
  }

  public Command scaledTargetCommand(
      Color color, DoubleSupplier currentValue, DoubleSupplier targetValue) {
    return run(() -> scaledTarget(color, currentValue.getAsDouble(), targetValue.getAsDouble()))
        .ignoringDisable(true);
  }

  public Command mappedCommand(DoubleSupplier currentValue) {
    return run(() -> mapped(currentValue.getAsDouble())).ignoringDisable(true);
  }

  public Command larsonCommand(Supplier<Color> color, double duration) {
    return run(() -> larson(color.get(), duration)).ignoringDisable(true);
  }
}
