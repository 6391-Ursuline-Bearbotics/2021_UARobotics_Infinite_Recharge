package frc.robot.UA6391;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Xbox6391 {
   public final XboxController m_Xbox6391Controller;

   public final Trigger POVUpish;
   public final Trigger POVDownish;
   public final Trigger POVLeftish;
   public final Trigger POVRightish;

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

   public Xbox6391 (int port) {
      m_Xbox6391Controller = new XboxController(port);

      POVUpish = new POVButton(m_Xbox6391Controller, 315).or(new POVButton(m_Xbox6391Controller, 0)).or(new POVButton(m_Xbox6391Controller, 45));
      POVDownish = new POVButton(m_Xbox6391Controller, 225).or(new POVButton(m_Xbox6391Controller, 180)).or(new POVButton(m_Xbox6391Controller, 135));
      POVLeftish = new POVButton(m_Xbox6391Controller, 225).or(new POVButton(m_Xbox6391Controller, 270)).or(new POVButton(m_Xbox6391Controller, 315));
      POVRightish = new POVButton(m_Xbox6391Controller, 45).or(new POVButton(m_Xbox6391Controller, 90)).or(new POVButton(m_Xbox6391Controller, 135));

      POVUp = new POVButton(m_Xbox6391Controller, 0);
      POVRight = new POVButton(m_Xbox6391Controller, 90);
      POVDown = new POVButton(m_Xbox6391Controller, 180);
      POVLeft = new POVButton(m_Xbox6391Controller, 270);

      XButton = new JoystickButton(m_Xbox6391Controller, XboxController.Button.kX.value);
      YButton = new JoystickButton(m_Xbox6391Controller, XboxController.Button.kY.value);
      AButton = new JoystickButton(m_Xbox6391Controller, XboxController.Button.kA.value);
      BButton = new JoystickButton(m_Xbox6391Controller, XboxController.Button.kB.value);

      BumperL = new JoystickButton(m_Xbox6391Controller, XboxController.Button.kBumperLeft.value);
      BumperR = new JoystickButton(m_Xbox6391Controller, XboxController.Button.kBumperRight.value);

      BackButton = new JoystickButton(m_Xbox6391Controller, XboxController.Button.kBack.value);
      StartButton = new JoystickButton(m_Xbox6391Controller, XboxController.Button.kStart.value);

      LeftStickButton = new JoystickButton(m_Xbox6391Controller, XboxController.Button.kStickLeft.value);
      RightStickButton = new JoystickButton(m_Xbox6391Controller, XboxController.Button.kStickRight.value);
   }
   
   public double JoystickLX() {
      return m_Xbox6391Controller.getX(GenericHID.Hand.kLeft);
   }

   public double JoystickLY() {
      return m_Xbox6391Controller.getY(GenericHID.Hand.kLeft);
   }

   public double JoystickRX() {
      return m_Xbox6391Controller.getX(GenericHID.Hand.kRight);
   }

   public double JoystickRY() {
      return m_Xbox6391Controller.getY(GenericHID.Hand.kRight);
   }

   public double TriggerL() {
      return m_Xbox6391Controller.getTriggerAxis(GenericHID.Hand.kLeft);
   }

   public double TriggerR() {
      return m_Xbox6391Controller.getTriggerAxis(GenericHID.Hand.kRight);
   }

   public void setLeftRumble(double value) {
      m_Xbox6391Controller.setRumble(GenericHID.RumbleType.kLeftRumble, value);
   }

   public void setRightRumble(double value) {
      m_Xbox6391Controller.setRumble(GenericHID.RumbleType.kRightRumble, value);
   }
}
