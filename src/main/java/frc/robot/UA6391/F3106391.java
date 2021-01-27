package frc.robot.UA6391;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class F3106391{
   public F310Controller m_F3106391Controller;

   public final Trigger POVUp;
   public final Trigger POVDown;
   public final Trigger POVLeft;
   public final Trigger POVRight;

   public final JoystickButton XButton;
   public final JoystickButton YButton;
   public final JoystickButton AButton;
   public final JoystickButton BButton;

   public final JoystickButton BumperL;
   public final JoystickButton BumperR;

   public final JoystickButton BackButton;
   public final JoystickButton StartButton;

   public final JoystickButton LeftStickButton;
   public final JoystickButton RightStickButton;

   public F3106391 (int port) {
      m_F3106391Controller = new F310Controller(port);

      POVUp = new POVButton(m_F3106391Controller, 315).or(new POVButton(m_F3106391Controller, 0)).or(new POVButton(m_F3106391Controller, 45));
      POVDown = new POVButton(m_F3106391Controller, 225).or(new POVButton(m_F3106391Controller, 180)).or(new POVButton(m_F3106391Controller, 135));
      POVLeft = new POVButton(m_F3106391Controller, 225).or(new POVButton(m_F3106391Controller, 270)).or(new POVButton(m_F3106391Controller, 315));
      POVRight = new POVButton(m_F3106391Controller, 45).or(new POVButton(m_F3106391Controller, 90)).or(new POVButton(m_F3106391Controller, 135));

      XButton = new JoystickButton(m_F3106391Controller, F310Controller.Button.kX.value);
      YButton = new JoystickButton(m_F3106391Controller, F310Controller.Button.kY.value);
      AButton = new JoystickButton(m_F3106391Controller, F310Controller.Button.kA.value);
      BButton = new JoystickButton(m_F3106391Controller, F310Controller.Button.kB.value);

      BumperL = new JoystickButton(m_F3106391Controller, F310Controller.Button.kBumperLeft.value);
      BumperR = new JoystickButton(m_F3106391Controller, F310Controller.Button.kBumperRight.value);

      BackButton = new JoystickButton(m_F3106391Controller, F310Controller.Button.kBack.value);
      StartButton = new JoystickButton(m_F3106391Controller, F310Controller.Button.kStart.value);

      LeftStickButton = new JoystickButton(m_F3106391Controller, F310Controller.Button.kStickLeft.value);
      RightStickButton = new JoystickButton(m_F3106391Controller, F310Controller.Button.kStickRight.value);
   }
   
   public double JoystickLX() {
      return m_F3106391Controller.getX(GenericHID.Hand.kLeft);
   }

   public double JoystickLY() {
      return m_F3106391Controller.getY(GenericHID.Hand.kLeft);
   }

   public double JoystickRX() {
      return m_F3106391Controller.getX(GenericHID.Hand.kRight);
   }

   public double JoystickRY() {
      return m_F3106391Controller.getY(GenericHID.Hand.kRight);
   }
}

