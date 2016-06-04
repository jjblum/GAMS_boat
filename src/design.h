#ifndef _DESIGN_H_
#define _DESIGN_H_

enum class DESIGN_TYPE
{
  LUTRA_TANK,
  LUTRA_ARTICULATED_FAN,
  LUTRA_TANK_FAN,
  SERVAL_CATAMARAN,
  FELIS_CATAMARAN
};

class Design
{
public:
  Design(DESIGN_TYPE type_);
  ~Design();
 
protected:
  DESIGN_TYPE type;

};


#endif // _DESIGN_H_
