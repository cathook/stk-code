namespace webcam {


ArcCosFunction::ArcCosFunction() :
    Function<double>(acos, [](double x) { return -1 / sqrt(1 - Squ(x)); }) {}


}  // namespace webcam
