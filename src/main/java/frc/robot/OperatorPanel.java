package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Keyhive Maypad operator panel — 5 rows × 4 columns, DS port 2.
 *
 * <p>
 * Use {@link #key(int, int)} to bind commands by physical position.
 * Row 0 is the top row; col 0 is the leftmost column.
 * Firmware: <a href="https://github.com/df1317/maypad-frc">df1317/maypad-frc</a>
 *
 * <pre>
 *        Col 0               Col 1               Col 2               Col 3
 * Row 0  [ intakeHome*     ] [                  ] [                  ] [               ]
 * Row 1  [ autoDistance    ] [ hoodHome*       ] [ hoodTest(T)    ] [ shootAll(T)   ]
 * Row 2  [ distAdvance     ] [ intakeFwd       ] [                ] [ shootFeed     ]
 * Row 3  [ distReduce      ] [ intakeRev       ] [                ] [ feedRev       ]
 * Row 4  [                 ] [                 ] [                ] [               ]
 *
 * * = works in both teleop and test    (T) = test mode only
 * </pre>
 */
public class OperatorPanel extends CommandGenericHID {

	public static final int ROWS = 5;
	public static final int COLS = 4;

	public OperatorPanel(int port) {
		super(port);
	}

	/**
	 * Returns the {@link Trigger} for the key at the given physical position.
	 *
	 * @param row
	 *          0-based row index, where 0 is the top row
	 * @param col
	 *          0-based column index, where 0 is the leftmost column
	 * @return trigger that is active while the key is held
	 */
	public Trigger key(int row, int col) {
		return button(row * COLS + col + 1); // WPILib buttons are 1-based
	}
}
