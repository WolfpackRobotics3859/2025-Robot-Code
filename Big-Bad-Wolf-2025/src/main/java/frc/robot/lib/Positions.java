package frc.robot.lib;

public class Positions 
{
    public enum Level
    {
        ONE(0),
        TWO(1),
        THREE(2),
        FOUR(3);

        private int value;

        Level(int value)
        {
            this.value = value;
        }

        public int getValue()
        {
            return this.value;
        }

        public static Level fromInt(int i) 
        {
            for (Level level : Level.values()) {
                if (level.getValue() == i) {
                    return level;
                }
            }
            throw new IllegalArgumentException("Unexpected value: " + i);
        }
    }

    public enum Face
    {
        ONE(0),
        TWO(1),
        THREE(2),
        FOUR(3),
        FIVE(4),
        SIX(5);

        private int value;

        Face(int value)
        {
            this.value = value;
        }

        public int getValue()
        {
            return this.value;
        }

        /**
         * Faces are 0th indexed, meaning face 1 is represented by 0.
         * @param i
         * @return
         */
        public static Face fromInt(int i) 
        {
            for (Face face : Face.values()) {
                if (face.getValue() == i) {
                    return face;
                }
            }
            throw new IllegalArgumentException("Unexpected value: " + i);
        }
    }

    public enum Side
    {
        LEFT(0),
        RIGHT(1);

        private int value;

        Side(int value)
        {
            this.value = value;
        }

        public int getValue()
        {
            return this.value;
        }

        public static Side fromInt(int i) 
        {
            if(i == 1)
            {
                return Side.RIGHT;
            }
            return Side.LEFT;
        }
    }
}
