#include "example/example.h"

namespace example
{
  std::string toString(OperationT op)
  {
    switch (op) {
      case OperationT::SUM :
        return "SUM";
      case OperationT::DIVISION :
        return "DIVISION";
      case OperationT::PRODUCT :
        return "PRODUCT";
      case OperationT::SUBSTRACTION :
        return "SUBSTRACTION";
      default :
        return "UNKNOWN";
    }
  }

  Example::Example(int initial_value)
  {
    _to_pub_data = initial_value;
  }

  void Example::process()
  {
    setPubData(getPubData() + _subbed_data);
  }

  float Example::calculate(const OperationT op, const float first, const float second)
  {
    std::cout << "Operation: " << toString(op) << std::endl;
    switch (op) {
      case OperationT::SUM :
        return getSum(first, second);
      case OperationT::DIVISION :
        return getDivision(first, second);
      case OperationT::PRODUCT :
        return getProduct(first, second);
      case OperationT::SUBSTRACTION :
        return getSubstraction(first, second);
      default :
        throw std::invalid_argument("calculate arguments: operation type unknown");
    }
  }

  float Example::getSum(const float first, const float second)
  {
    return first + second;
  }

  float Example::getSubstraction(const float first, const float second)
  {
    return first - second;
  }

  float Example::getDivision(const float first, const float second)
  {
    return first / second;
  }

  float Example::getProduct(const float first, const float second)
  {
    return first * second;
  }

} // namespace example