package org.usfirst.frc.team7230.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class EncoderPIDSource implements PIDSource {
	private Encoder m_enc_0;
	private Encoder m_enc_1;
	double TrackWidth = 27.75;
	public EncoderPIDSource(Encoder enc_0, Encoder enc_1) {
		m_enc_0 = enc_0;
		m_enc_1 = enc_1;
	}
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub

	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		return (m_enc_1.getDistance() - m_enc_0.getDistance())/TrackWidth;
	}

}

