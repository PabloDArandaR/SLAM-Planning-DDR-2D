#ifndef SCANNER_METADATA_GUARD
#define SCANNER_METADATA_GUARD

class scanerMetadata {
  public:
    scanerMetadata();

    void set_angledata(float angle_increment, float angle_min, float angle_max);
    void set_rangedata(float range_min, float range_max);

    float get_angleincrement();
    float get_anglemin();
    float get_anglemax();

    float get_rangemin();
    float get_rangemax();

  private:
    float angle_increment, angle_min, angle_max;
    float range_min, range_max;
};

#endif