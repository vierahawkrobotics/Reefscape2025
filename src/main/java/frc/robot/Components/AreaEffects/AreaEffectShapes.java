package frc.robot.Components.AreaEffects;

/**
 * Shapes used for area effect targeting/designation
 * @author Darren Ringer
 */

public class AreaEffectShapes {

    public class Point{
        public double x;
        public double y;
        public Point(double x, double y){
            this.x = x;
            this.y = y;
        }
    }


    /**
     * Parent generic class for area effects
     */
    public class DefaultShape{
        /**
         * Checks if (x,y) is inside the effect
         * @param point Point (x,y) to check
         * @return Boolean if the point is inside the effect
         */
        public boolean check(double x, double y){
            return false;
        }
        /**
         * Checks if a point is inside the effect
         * @param point Point to check
         * @return Boolean if the point is inside the effect
         */
        public boolean check(Point point){
            return false;
        }
    }


    /**
     * Area effect circle shape
     * 
     */
    public class Circle extends DefaultShape{
        private Point center;
        private double r;
        
        public Circle(double x, double y, double r){
            center = new Point(x, y);
            this.r = r;
        }

        @Override
        public boolean check(double x, double y){
            return Math.sqrt((center.x - x)*(center.x - x) + (center.y - y)*(center.y - y)) <= r;
        }

        @Override
        public boolean check(Point point){
            return Math.sqrt((center.x - point.x)*(center.x - point.x) + (center.y - point.y)*(center.y - point.y)) <= r;
        }
    }



    public class Polygon extends DefaultShape{
        private Point[] pointList;

        public Polygon(Point[] pointList){
            this.pointList = pointList;
        }
        
        public Polygon(double[] xList, double[] yList){
            this.pointList = new Point[Math.min(xList.length,yList.length)];
            for(int i=0;i<Math.min(xList.length, yList.length);i++){
                this.pointList[i] = new Point(xList[i], yList[i]);
            }
        }

        @Override
        public boolean check(double x, double y){
            int windingNumber = 0;
            Point v1, v2;
            double xIntersect;
            for(int i=0;i<pointList.length;i++){
                v1 = pointList[i];
                v2 = pointList[(i+1) % pointList.length];

                if(Math.min(v1.y, v2.y) < y && Math.max(v1.y, v2.y) > y){
                    xIntersect = v1.x + (y-v1.y)*(v1.x-v2.x)/(v1.y-v2.y);
                    if(xIntersect < x){
                        if(v1.y > v2.y) windingNumber++;
                        else windingNumber--;
                    }
                }
            }
            return windingNumber != 0;
        }

        @Override
        public boolean check(Point point){
            double x = point.x;
            double y = point.y;

            int windingNumber = 0;
            Point v1, v2;
            double xIntersect;
            for(int i=0;i<pointList.length;i++){
                v1 = pointList[i];
                v2 = pointList[(i+1) % pointList.length];

                if(Math.min(v1.y, v2.y) < y && Math.max(v1.y, v2.y) > y){
                    xIntersect = v1.x + (y-v1.y)*(v1.x-v2.x)/(v1.y-v2.y);
                    if(xIntersect < x){
                        if(v1.y > v2.y) windingNumber++;
                        else windingNumber--;
                    }
                }
            }
            return windingNumber != 0;
        }
    }
}
