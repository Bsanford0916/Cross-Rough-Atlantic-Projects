// Configuration menu stuff
#define MENU_ITEMS 16  // was 14
MenuItem* MenuItems[MENU_ITEMS];

// ================================ Builds the config menu structure using my MenuHelpers, a tricky bit of code this is ===========================
void BuildMenu() {
  MenuItems[0] = new ExitMenuItem();
  strcpy(MenuItems[0]->Title, "EXIT");

  MenuItems[1] = new MotorMenuItem();
  strcpy(MenuItems[1]->Title, "Power :");

  MenuItems[2] = new ROFMenuItem();
  strcpy(MenuItems[2]->Title, "A ROF :");

  MenuItems[3] = new ROFMenuItem();
  strcpy(MenuItems[3]->Title, "B ROF :");

  MenuItems[4] = new BurstSizeMenuItem();
  strcpy(MenuItems[4]->Title, "Burst :");

  MenuItems[5] = new MagSizeMenuItem();
  strcpy(MenuItems[5]->Title, "Mag S :");

  MenuItems[6] = new TimingMenuItem();
  strcpy(MenuItems[6]->Title, "Ramp U:");

  MenuItems[7] = new TimingMenuItem();
  strcpy(MenuItems[7]->Title, "Ramp D:");

  MenuItems[8] = new TimingMenuItem();
  strcpy(MenuItems[8]->Title, "Dwel U:");

  MenuItems[9] = new TimingMenuItem();
  strcpy(MenuItems[9]->Title, "Dwel D:");

  MenuItems[10] = new SolenoidTimingMenuItem();
  strcpy(MenuItems[10]->Title, "SP Hi :");

  MenuItems[11] = new SolenoidTimingMenuItem();
  strcpy(MenuItems[11]->Title, "SP Low:");

  MenuItems[12] = new SolenoidTimingMenuItem();
  strcpy(MenuItems[12]->Title, "SP Ret:");

  MenuItems[13] = new BatteryMenuItem();
  strcpy(MenuItems[13]->Title, "BatType");

  MenuItems[14] = new BatOffsetMenuItem();
  strcpy(MenuItems[14]->Title, "Bat Off");

  // ----- Referance code Does buttons -------------------
  //MenuItems[13] = new ActionMenuItem();
  //strcpy( MenuItems[13]->Title, "Cen L:" );

  //MenuItems[14] = new ActionMenuItem();
  //strcpy( MenuItems[14]->Title, "Rgt S:" );

  //MenuItems[15] = new ActionMenuItem();
  //strcpy( MenuItems[15]->Title, "Rgt L:" );

  //MenuItems[16] = new ActionMenuItem();
  //strcpy( MenuItems[16]->Title, "ROT L:" );

  MenuItems[15] = new ExitMenuItem();
  strcpy(MenuItems[15]->Title, "EXIT");
}

