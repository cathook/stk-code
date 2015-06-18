#ifndef WEBCAM_FUNCTION_HPP
#define WEBCAM_FUNCTION_HPP

#include <math.h>

#include <functional>
#include <memory>


namespace webcam {


template <typename Input>
class FunctionBase {
 public:
  virtual ~FunctionBase() {}

  virtual double GetValue(Input x) const = 0;

  virtual Input GetDerivValue(Input x) const = 0;

  virtual FunctionBase *Clone() const = 0;

 protected:
  FunctionBase() {}
};


template <typename Input>
class TrivialFunction : public FunctionBase<Input> {
 public:
  TrivialFunction(std::function<double(Input)> eval_func,
                  std::function<Input(Input)> deriv_func) :
      eval_func_(eval_func), deriv_func_(deriv_func) {}

  double GetValue(Input x) const { return eval_func_(x); }

  Input GetDerivValue(Input x) const { return deriv_func_(x); }

  FunctionBase<Input> *Clone() const {
    return new TrivialFunction(eval_func_, deriv_func_);
  }

 private:
  std::function<double(Input)> eval_func_;
  std::function<Input(Input)> deriv_func_;
};


template <typename Input>
class BinaryAddFunction : public FunctionBase<Input> {
 public:
  BinaryAddFunction(FunctionBase<Input> *f1, FunctionBase<Input> *f2) :
      f1_(f1), f2_(f2) {}

  double GetValue(Input x) const {
    return f1_->GetValue(x) + f2_->GetValue(x);
  }

  Input GetDerivValue(Input x) const {
    return f1_->GetDerivValue(x) + f2_->GetDerivValue(x);
  }

  FunctionBase<Input> *Clone() const {
    return new BinaryAddFunction(f1_->Clone(), f2_->Clone());
  }

 private:
  std::unique_ptr<FunctionBase<Input>> f1_;
  std::unique_ptr<FunctionBase<Input>> f2_;
};


template <typename Input>
class BinaryMultFunction : public FunctionBase<Input> {
 public:
  BinaryMultFunction(FunctionBase<Input> *f1, FunctionBase<Input> *f2) :
      f1_(f1), f2_(f2) {}

  double GetValue(Input x) const {
    return f1_->GetValue(x) * f2_->GetValue(x);
  }

  Input GetDerivValue(Input x) const {
    return (f1_->GetDerivValue(x) * f2_->GetValue(x) +
            f2_->GetDerivValue(x) * f1_->GetValue(x));
  }

  FunctionBase<Input> *Clone() const {
    return new BinaryMultFunction(f1_->Clone(), f2_->Clone());
  }

 private:
  std::unique_ptr<FunctionBase<Input>> f1_;
  std::unique_ptr<FunctionBase<Input>> f2_;
};


template <typename Input>
class BinaryPipeFunction : public FunctionBase<Input> {
 public:
  BinaryPipeFunction(FunctionBase<double> *f1, FunctionBase<Input> *f2) :
      f1_(f1), f2_(f2) {}

  double GetValue(Input x) const {
    return f1_->GetValue(f2_->GetValue(x));
  }

  Input GetDerivValue(Input x) const {
    double m = f1_->GetDerivValue(f2_->GetValue(x));
    return f2_->GetDerivValue(x) * m;
  }

  FunctionBase<Input> *Clone() const {
    return new BinaryPipeFunction(f1_->Clone(), f2_->Clone());
  }

 private:
  std::unique_ptr<FunctionBase<double>> f1_;
  std::unique_ptr<FunctionBase<Input>> f2_;
};


template <typename Input>
class Function : public FunctionBase<Input> {
 public:
  Function(const Function &inst) :
      func_(inst.func_->Clone()), k_(inst.k()), n_(inst.n()) {}

  Function(std::function<double(Input)> eval_func,
           std::function<Input(Input)> deriv_func,
           double k = 1, double n = 1) :
      func_(new TrivialFunction<Input>(eval_func, deriv_func)), k_(k), n_(n) {}

  Function(FunctionBase<Input> *ptr) : func_(ptr), k_(1), n_(1) {}

  virtual ~Function() {}

  double k() const { return k_; }

  double set_k(double val) { k_ = val; return k(); }

  double n() const { return n_; }

  double set_n(double val) { n_ = val; return n(); }

  double GetValue(Input x) const {
    return k_ * pow(func_->GetValue(x), n_);
  }

  Input GetDerivValue(Input x) const {
    double m = k_ * n_ * pow(func_->GetValue(x), n_ - 1);
    return func_->GetDerivValue(x) * m;
  }

  FunctionBase<Input> *Clone() const {
    return new Function(*this);
  }

  double operator()(Input x) const { return GetValue(x); }

  Function operator+(const Function &f2) const {
    return Function(new BinaryAddFunction<Input>(Clone(), f2.Clone()));
  }

  Function operator-(const Function& f2) const {
    Function *inst = new Function(f2);
    inst->set_k(-(inst->k()));
    return Function(new BinaryAddFunction<Input>(Clone(), inst));
  }

  Function operator*(const Function &f2) const {
    return Function(new BinaryMultFunction<Input>(Clone(), f2.Clone()));
  }

  Function operator/(const Function &f2) const {
    Function *inst = new Function(f2);
    inst->set_n(-(inst->n()));
    return Function(new BinaryMultFunction<Input>(Clone(), inst));
  }

  template <typename Input2>
  Function<Input2> operator|(const Function<Input2> &f2) const {
    return Function<Input2>(
        new BinaryPipeFunction<Input2>(Clone(), f2.Clone()));
  }

  Function &operator=(const Function &func) {
    func_.reset(func.func_->Clone());
    set_k(func.k());
    set_n(func.n());
    return *this;
  }

 private:
  std::unique_ptr<FunctionBase<Input>> func_;
  double k_;
  double n_;
};

}  // namespace webcam


#endif  // WEBCAM_FUNCTION_HPP
