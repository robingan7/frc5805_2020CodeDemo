
package frc.lib.waypoint;

/**
 * 
 * @param <T> type you want to interpolate
 */
public interface InterpolateSingle<T>{
    /**
     * interpolate between this value and the other one
     * if this value is 1, method returns the other value 
     * if the value is 0, method return this value
     * in other cases, method return value between those two proportionally
     *
     * @param other high limit
     * @param request value between 0 and 1
     * @param the result of interplating
     */

     public T interpolate(T other, double request);
}