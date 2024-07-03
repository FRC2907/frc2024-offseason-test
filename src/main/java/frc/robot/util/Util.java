
package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class Util {
  //198 instructions
  public static int[] fromHSV_slow(short h, short s, short v) {
    final int region = (h / 30) % 6;
    final int chroma = (s * v) / 255;
    final int m = v - chroma;
    final int X = (chroma * ((h % 30) * 255 / 30)) >> 8;

    switch (region) {
        case 0:
            return new int[]{v, X + m, m};
        case 1:
            return new int[]{v - X, v, m};
        case 2:
            return new int[]{m, v, X + m};
        case 3:
            return new int[]{m, v - X, v};
        case 4:
            return new int[]{X + m, m, v};
        default:
            return new int[]{v, m, v - X};
    }
}
  
// made more perf better from this
  // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibj/src/main/java/edu/wpi/first/wpilibj/util/Color.java
  //https://godbolt.org/#z:OYLghAFBqd5TKALEBjA9gEwKYFFMCWALugE4A0BIEAZgQDbYB2AhgLbYgDkAjF%2BTXRMiAZVQtGIHgBYBQogFUAztgAKAD24AGfgCsp5eiyahdLAG4tyKxqiIEh1ZpgDC6egFc2TEAGYt5M4AMgRM2AByXgBG2KR%2B5AAO6ErEDkxunt5%2BAUkp9kIhYZFsMXG%2B1ti2%2BUwiRCykRBlePv4VVWm19USFEdGx8Up1DU1ZrYNdPcWlfgCU1ugepKicXAD0qwDUACoAngnYGzsLpBsYOBtIsdjkG2Qb9OgsmBvGG9jq7AmMAHQApFoAQVQRiUSg2IgAjh56gdfgB2ABC/wBGw2CQ8UXoBFQG3G9hxoSIvwArEjiQARDY0UjoNgACREADUIEokGQiBcbqz2biuWyGhtzDMNvCkYDUai6Kx6BtCRtSNhgGkRb5KRAkBtNv5hb8AEzEjYANl%2BvjFKIlUoksuEpyQNLYLBVarBACpBcLNvriSazRKqaErXK2E7BRsALS2%2B0sH3Iv2WmVygAaIYgqDttMdbuQIv1G21GzdXs1ea0Mx1vlwJtwGwAHDHAbGJUoAO7ENMbCAKpVCHWIxt%2B1HiFQbLQgfsDv0KoiLJgbMLN61E0kk8mi8w3ZN6hEbNg3Njw1em8d%2BocHHhj8UTgdTmdz7ALwkksmrxHmcMbRM3dc7g/180T08Nl1C9/yvVEb1IWd50XJ8V1FXdBQ3HNt33OFD19K9AN8ECwMnbBp0gu8H2EWCKXgr930/QVfyPS8AJYYdpBw3DwPw29oMfZcyMRTddRQvcvxojCJxwGgWA8egiGYliIKg%2B8YK4l8EW/BC3wjRMhPHX8GzQxtkQPLg5nobhiX4HxeH4dBuAAKQsR0lGOZYc18PhyCkrgdDLcgAGsQGJAJjK4aR%2BDYPyAnMnRyCsrh%2BCUEAAnczzyDgWAUDOPBCBICgqFoBhmHYFZXMEYQxAkTgZDkYRlDUTQPL0EBdUMYxTDstpsDsNInCYVx3GaAxglCXoSn6XVDUSZJUiEEYfB4ca8jSSY%2BjiUa2o6oROmGXqslmmx2uqDbukGqYRrG8ZNsyGbrCGQ6iiWhrDTmBzFmWbh1m2PYDiORZTiwA5LgVG47geJ4Xlnd5Ph%2BZFgQYsFIWhBURT7OjUXRTFsVxOp8QU58qXtBlmW5AUkD5HklBJgUhUR4SJXWeNF0jDMU1dd1iy9H0wOPf1pXp4MTUpNSO2ZynPWJb1TRFZHJQDBMbU3VUO05ljURZAsWZFg1VcVpWIEJGYAFkWCIJBvhpDxuuzPUDXzLMiy1LRvlLGZOfLSsK1rP9OZbNsNQgDtdfVYt80to0NmFVFRS1wDRy11iCLk4il2fNckK3Hc900yWJUA88Y/lNjCI4kjFLXSiv3T3TaNAk8GIOYDc9kojsbgxFVJTvifwr6nq%2BHbD6/z%2BOm%2B4hFVNL6jO8jmuNiYvu48bzik545C0%2BX8wM6riVRPEyTpKvBvC8T5vlIE0N1LXidtPNC/9LQwzDBMsztEsmy7NxRzYV1XxdX4RLDLmS4nn6BAIy3BgrkFCv5cgEUn4xWsPFNyj8vK%2BWJL4b4cIay%2BAAJw1g/oaYkGCtBwmkI1QKvgH51Sitwb%2BCDkqIHVGgWkCQ8rZWoBgNgjDGBxDMJYXUPAtCNToJJWIcUIBREfuQKIAZSA7G4K5CRrApEAHkoi6D2jI/grCODCAUUweg0jyE4CiB4YALgJD0DihZcgOAHQmEkPoggCoOrmGwOYyK7x2oeCIIVfghJKhiKxFEUg9QdhuBwGIogpACChQsU40gURkjYHJNgaxwAsQmGoTQIwwAlCMgIPeBR%2BxzJFXkKVSQFViqKBUBoMR%2BhZpGDSVwqw/i4qQDmOgBI1RzGWRiREnAzSgGrWqF1HqF1%2BrdUWsNOIs1ciTXSFtS60zqjjOmDtSoe0OjXWmgYXaa0ajXSWf0HaGy5lbL2UdO6PBHpvykMArgplIFiOihsXQmBvJAR4A7b4uoOz4GIHcPULkZhULqnMOhrD2GxEoCwhhTCQANJ4XwgQDBPGkGEaI8hciglqPEZInYSiVF2CxRo5gRBtG6LEQYoxJj6BmKxVY5qtjIqEAcfYJxLj%2BBuNQB4rxlBhC%2BPIf4wJUiQkrEiuEyJWKYlxJUIk5JqTQDAoEJk7JuTmz5OYFi8pJTyqyHKdVKp5D9DEOarC1qTT4CtPaWkTpUVukEF6eagZnUIDOE2bNAat0JkGAWWkV1c0Zn7MmY69aRyRkrPaMGiYZzPWHK6L6s6N0hrLMuc9Tguobl3KgRQrgTyXlvI%2BV8iAPysrOTTUCpKdDCA0BoJC2gxTxClJ1fIPVtVIr0CQHFWp7bK00CIB9OBbaVDqHCSwNpgyYGkHbQ1cgE6lDdt7fsOBM73jDtHZ1GK6ayGRWiuSAgVaNg5PvLEHNryeH5sFGCZ5J73n2y/vA4Ff9sAALiP03y0hDSfJ4IaLQPA4SGl8NILQGD9Q1jvkFEKUgtDhQeZQ2BjUf5eUCrezN0Uy2/3IDElIjhpBAA%3D%3D
  // 189 instructions faster
  public static int[] fromHSV(short h, short s, short v) {
      //final int chroma = (s * v) / 255;        
      final int m = v - (s * v) / 255; 
      final int X = (
                  (s * v) / 255  * 
                  (int)Math.round((h % 30) * (255 / 30.0))
      ) >> 8;

      switch ( (int)(h / 30) % 6 )   {
        case 0:
          return new int[]{v, X + m, m};
        case 1:
          return new int[]{v - X, v, m};
        case 2:
          return new int[]{m, v, X + m};
        case 3:
          return new int[]{m, v - X, v};
        case 4:
          return new int[]{X + m, m, v};
        default:
          return new int[]{v, m, v - X};
      }
  }

  public static TalonFX createTalonFXGroup(int[] ids, boolean invert) {
		if (ids.length == 0) {
			System.err.println("[EE] Attempted to create empty group of CANSparkMax");
			new Exception().printStackTrace();
		}
		TalonFX[] mcs = new TalonFX[ids.length];
		for (int i = 0; i < ids.length; i++) {
			mcs[i] = new TalonFX(ids[i]);
			if (i > 0)
				mcs[i].setControl(new Follower(ids[0], false));
		}
		mcs[0].setInverted(invert);
		return mcs[0];
	}

  public static TalonSRX createTalonSRXGroup(int[] ids, boolean invert) {
		if (ids.length == 0) {
			System.err.println("[EE] Attempted to create empty group of CANSparkMax");
			new Exception().printStackTrace();
		}
		TalonSRX[] mcs = new TalonSRX[ids.length];
		for (int i = 0; i < ids.length; i++) {
			mcs[i] = new TalonSRX(ids[i]);
			if (i > 0)
				mcs[i].follow(mcs[0], FollowerType.PercentOutput);
		}
		mcs[0].setInverted(invert);
		return mcs[0];
	}

}
