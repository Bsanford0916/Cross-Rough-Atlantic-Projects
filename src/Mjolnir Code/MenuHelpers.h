// Menu object types
// Just chucked into a header because I don't want to have to have this at the top of the INO file :p

// This does range testing on the Config screen Values, Oled display

class MenuItem
{
  public:
  int CurrentValue = 0;
  char Title[8] = "\0";
  char Output[8] = "\0";

  virtual void Increase(  ) = 0;
  virtual void Decrease(  ) = 0; 
  virtual void PrepareOutput( ) = 0;
};

// ------------------------------------------------------------------------------
class MotorMenuItem : public MenuItem       //Power output Menu item 1
{
  private:
    
    
  public:

  void Increase()
  {
    CurrentValue += 5;
    if( CurrentValue > 100 ) CurrentValue = 100;
  }
  void Decrease()
  {
    CurrentValue -= 5;
    if( CurrentValue < 30 ) CurrentValue = 30;   
  }
  void PrepareOutput()
  {
    sprintf( Output, "%3d%%", CurrentValue ); 
  }
};

// -------------------------------------------------------------------------
class MagSizeMenuItem : public MenuItem       //Mag Size menu item 5
{
  private:
    
    
  public:

  void Increase()
  {
    if( CurrentValue == 0 )
      CurrentValue = 6;
    else if( CurrentValue < 100 )
      CurrentValue ++;
  }
  void Decrease()
  {
    if( CurrentValue == 6 ) 
    {
      CurrentValue = 0;
    }
    else if( CurrentValue > 6 )
    {
      CurrentValue --;
    }
  }

  void PrepareOutput()
  {
    if( CurrentValue == 0 )  
    {
      strcpy( Output, "  UP" );
    }
    else
    {
      sprintf( Output, "%4d", CurrentValue );
    }
  }
};

// --------------------------------------------------------------------------------
class BurstSizeMenuItem : public MenuItem     //Burst Size menu Item 4
{
  private:
    
    
  public:

  void Increase()
  {
    if( CurrentValue < 49 )
      CurrentValue ++;
  }
  void Decrease()
  {
    if( CurrentValue > 1 )
      CurrentValue --;
  }
  void PrepareOutput()
  {
    sprintf( Output, "%4d", CurrentValue ); 
  }
};

// ----------------------------------------------------------------
class ROFMenuItem : public MenuItem         //ROF
{
  private:
    
    
  public:

  void Increase()
  {
    if( CurrentValue < 150 )
      CurrentValue += 15;
  }
  void Decrease()
  {
    if( CurrentValue > 0 )
      CurrentValue -= 15 ;
      
  }
  void PrepareOutput()
  {
    if( CurrentValue == 0 )
    {
      strcpy( Output, " MAX" );
    }
    else
    {
      sprintf( Output, "%4d", CurrentValue ); 
    }
  }
};

// ----------------------------------------------------------------------------
class TimingMenuItem : public MenuItem                //Timing
{
  private:
    
    
  public:

  void Increase()
  {
    CurrentValue += 100;
    if( CurrentValue > 5000 )
      CurrentValue = 5000;
  }
  void Decrease()
  {
    CurrentValue -= 100;
    if( CurrentValue < 0 )
      CurrentValue = 0;
  }
  void PrepareOutput()
  {
    sprintf( Output, "%4d", CurrentValue );
  }
};

//-------------------------------------------------------------------------------------------
class SolenoidTimingMenuItem : public MenuItem       //Solenoide Timing Menu Item
{
  private:
    
    
  public:

  void Increase()
  {
    if( CurrentValue == 0 )
      CurrentValue = 1;
    else if( CurrentValue < 1000 )
      CurrentValue ++;
  }
  void Decrease()
  {
    if( CurrentValue == 0 ) 
    {
      CurrentValue = 0;
    }
    else if( CurrentValue > 0 )
    {
      CurrentValue --;
    }
  }

  void PrepareOutput()
  {
    if( CurrentValue == 0 )  
    {
      strcpy( Output, "  UP" );
    }
    else
    {
      sprintf( Output, "%4d", CurrentValue );
    }
  }
};

// -------------------------------------------------------------------------------------------
class ActionMenuItem : public MenuItem        //action menu item Buttons 
{
  private:
   
    
  public:

  void Increase()
  {
    CurrentValue ++;
    if( CurrentValue > 6 )
      CurrentValue = 0;
  }
  void Decrease()
  {
    CurrentValue --;
    if( CurrentValue < 0 )
      CurrentValue = 6;
  }
  void PrepareOutput()
  {
    switch( CurrentValue )
    {
      case 0:
        strcpy( Output, "Single" );
        break;
      case 1:
        strcpy( Output, "Burst" );
        break;
      case 2:
        strcpy( Output, "Auto" );
        break;
      case 3:
        strcpy( Output, "Safe" );
        break;
      case 4:
        strcpy( Output, "Pro A" );
        break;
      case 5: 
        strcpy( Output, "Pro B" );
        break;
      case 6: default:
        strcpy( Output, "Null" );
        break;
    }    
  }
};

// ---------------------------------------------------------------------
class ExitMenuItem : public MenuItem
{
  private:
    char Buffer[0];
    
  public:

  void Increase()
  {
  }
  void Decrease()
  {
  }
  void PrepareOutput()
  {
  }
};
