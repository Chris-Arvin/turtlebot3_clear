/*
 * Note：定义了如果要在(x0,y0)和（x1，y1）间填充的话，要怎么做。
 */
#ifndef LINE_ITERATOR_H
#define LINE_ITERATOR_H

#include <stdlib.h>

namespace base_local_planner
{

/** An iterator implementing Bresenham Ray-Tracing. */
class LineIterator
{
public:
  //在(x0,y0)和(x1,y1)两点间画线，确定他们之间的迭代方式和画点的个数
  //0和1是起点和终点，x_和y_是当前点（可能是起终或中间点）
  LineIterator( int x0, int y0, int x1, int y1 )
    : x0_( x0 )
    , y0_( y0 )
    , x1_( x1 )
    , y1_( y1 )
    , x_( x0 ) // X and Y start of at first endpoint.
    , y_( y0 )
    , deltax_( abs( x1 - x0 ))
    , deltay_( abs( y1 - y0 ))
    , curpixel_( 0 )
  {
    //这四个是deltax=deltay的情况，只需要比较两点的方位就可以了
    if( x1_ >= x0_ )                 // The x-values are increasing
    {
      xinc1_ = 1;
      xinc2_ = 1;
    }
    else                          // The x-values are decreasing
    {
      xinc1_ = -1;
      xinc2_ = -1;
    }

    if( y1_ >= y0_)                 // The y-values are increasing
    {
      yinc1_ = 1;
      yinc2_ = 1;
    }
    else                          // The y-values are decreasing
    {
      yinc1_ = -1;
      yinc2_ = -1;
    }

    //纵向和横向，选择长的那一个作为总中间点的个数，短的那一个作为每两个点间的间隔
    if( deltax_ >= deltay_ )         // There is at least one x-value for every y-value
    {
      xinc1_ = 0;                  // Don't change the x when numerator >= denominator
      yinc2_ = 0;                  // Don't change the y for every iteration
      den_ = deltax_;
      num_ = deltax_ / 2;
      numadd_ = deltay_;
      numpixels_ = deltax_;         // There are more x-values than y-values
    }
    else                          // There is at least one y-value for every x-value
    {
      xinc2_ = 0;                  // Don't change the x for every iteration
      yinc1_ = 0;                  // Don't change the y when numerator >= denominator
      den_ = deltay_;
      num_ = deltay_ / 2;
      numadd_ = deltax_;
      numpixels_ = deltay_;         // There are more y-values than x-values
    }
  }
  
  //是否可以继续迭代
  bool isValid() const
  {
    return curpixel_ <= numpixels_;
  }

  void advance()
  {
    num_ += numadd_;              // Increase the numerator by the top of the fraction
    if( num_ >= den_ )            // Check if numerator >= denominator
    {
      num_ -= den_;               // Calculate the new numerator value
      x_ += xinc1_;               // Change the x as appropriate
      y_ += yinc1_;               // Change the y as appropriate
    }
    x_ += xinc2_;                 // Change the x as appropriate
    y_ += yinc2_;                 // Change the y as appropriate

    curpixel_++;
  }
  
  int getX() const { return x_; }
  int getY() const { return y_; }

  int getX0() const { return x0_; }
  int getY0() const { return y0_; }

  int getX1() const { return x1_; }
  int getY1() const { return y1_; }

private:
  int x0_; ///< X coordinate of first end point.
  int y0_; ///< Y coordinate of first end point.
  int x1_; ///< X coordinate of second end point.
  int y1_; ///< Y coordinate of second end point.

  int x_; ///< X coordinate of current point.
  int y_; ///< Y coordinate of current point.

  int deltax_; ///< Difference between Xs of endpoints.
  int deltay_; ///< Difference between Ys of endpoints.

  int curpixel_; ///< index of current point in line loop.

  int xinc1_, xinc2_, yinc1_, yinc2_;
  int den_, num_, numadd_, numpixels_;
};

} // end namespace base_local_planner

#endif // LINE_ITERATOR_H
