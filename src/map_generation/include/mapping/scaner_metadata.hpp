class scanerMetadata {
  public:
    scanerMetadata();

    void set_angledata(float, float, float);
    void set_rangedata(float, float);

    float get_angleincrement();
    float get_anglemin();
    float get_anglemax();

    float get_rangemin();
    float get_rangemax();

  private:
    float angle_increment, angle_min, angle_max;
    float range_min, range_max;
};