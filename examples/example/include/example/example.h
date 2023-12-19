#pragma once

#include <iostream>
#include <vector>
#include <stdexcept>

namespace example
{
  enum OperationT { SUM, DIVISION, PRODUCT, SUBSTRACTION, UNKNOWN };

  std::string toString(OperationT op);

  class Example
  {
  private:
    int _subbed_data;
    int _to_pub_data;

    inline void setPubData(const int data) { _to_pub_data = data; }

  public:
    Example(int initial_value);

    //
    inline void setSubbedData(const int data) { _subbed_data = data; }
    inline int getSubbedData() const { return _subbed_data; }
    inline int getPubData() const { return _to_pub_data; }

    //
    inline void reset() { _to_pub_data = 0; }
    void process();

    //
    float calculate(const OperationT op, const float first, const float second);
    float getSum(const float first, const float second);
    float getSubstraction(const float first, const float second);
    float getDivision(const float first, const float second);
    float getProduct(const float first, const float second);
  };

} // namespace example
