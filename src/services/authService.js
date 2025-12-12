import { supabase } from '../lib/supabaseClient';

class AuthService {
  constructor() {
    this.currentUser = null;
  }

  // Sign up with onboarding data
  async signUp(email, password, onboardingData) {
    try {
      const { data, error } = await supabase.auth.signUp({
        email,
        password,
        options: {
          data: {
            ...onboardingData,
            email
          }
        }
      });

      if (error) throw error;

      // Update the profiles table with onboarding data
      if (data.user) {
        await this.updateUserProfile(data.user.id, onboardingData);
      }

      this.currentUser = data.user;
      return data;
    } catch (error) {
      console.error('SignUp Error:', error);
      throw error;
    }
  }

  // Sign in
  async signIn(email, password) {
    try {
      const { data, error } = await supabase.auth.signInWithPassword({
        email,
        password
      });

      if (error) throw error;

      this.currentUser = data.user;
      return data;
    } catch (error) {
      console.error('SignIn Error:', error);
      throw error;
    }
  }

  // Sign out
  async signOut() {
    try {
      const { error } = await supabase.auth.signOut();
      if (error) throw error;
      this.currentUser = null;
    } catch (error) {
      console.error('SignOut Error:', error);
      throw error;
    }
  }

  // Get current user
  getCurrentUser() {
    return this.currentUser;
  }

  // Update user profile
  async updateUserProfile(userId, profileData) {
    try {
      const { data, error } = await supabase
        .from('profiles')
        .upsert([
          {
            id: userId,
            ...profileData,
            updated_at: new Date().toISOString()
          }
        ], { onConflict: 'id' });

      if (error) throw error;
      return data;
    } catch (error) {
      console.error('Update Profile Error:', error);
      throw error;
    }
  }

  // Get user profile
  async getUserProfile(userId) {
    try {
      const { data, error } = await supabase
        .from('profiles')
        .select('*')
        .eq('id', userId)
        .single();

      if (error) throw error;
      return data;
    } catch (error) {
      console.error('Get Profile Error:', error);
      throw error;
    }
  }

  // Listen for auth state changes
  onAuthStateChange(callback) {
    return supabase.auth.onAuthStateChange((event, session) => {
      if (session) {
        this.currentUser = session.user;
      } else {
        this.currentUser = null;
      }
      callback(event, session);
    });
  }
}

export const authService = new AuthService();