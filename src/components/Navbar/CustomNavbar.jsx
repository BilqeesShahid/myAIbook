import React from 'react';
import { useNavbarMobileSidebar } from '@docusaurus/theme-common';
import NavbarItem from '@theme/NavbarItem';
import UserProfile from './UserProfile';

// Create a custom navbar that includes the user profile
const CustomNavbar = (props) => {
  const mobileSidebar = useNavbarMobileSidebar();

  return (
    <nav
      className="navbar navbar--fixed-top"
      {...props}
    >
      <div className="navbar__inner">
        <div className="navbar__items">
          {/* Left items */}
          {props.items
            .filter(item => item.position === 'left')
            .map((item, idx) => (
              <NavbarItem
                {...item}
                key={idx}
              />
            ))}
        </div>

        <div className="navbar__items navbar__items--right">
          {/* User profile component for authenticated users */}
          <UserProfile />

          {/* Other right items except the user profile container */}
          {props.items
            .filter(item => item.position === 'right' && !item.value?.includes('user-profile-container'))
            .map((item, idx) => (
              <NavbarItem
                {...item}
                key={idx}
              />
            ))}
        </div>
      </div>

      {mobileSidebar && (
        <div className="navbar-sidebar__items">
          <div className="navbar-sidebar__item menu">
            {props.items.map((item, idx) => (
              <NavbarItem
                {...item}
                key={idx}
              />
            ))}
          </div>
        </div>
      )}
    </nav>
  );
};

export default CustomNavbar;