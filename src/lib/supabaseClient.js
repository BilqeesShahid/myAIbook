import { createClient } from '@supabase/supabase-js';

// Fallback values for development - these should be replaced with environment variables in production
const SUPABASE_URL = process.env.DOCUSAURUS_SUPABASE_URL || 'https://aotysxctwbzdwakbcngr.supabase.co';
const SUPABASE_ANON_KEY = process.env.DOCUSAURUS_SUPABASE_ANON_KEY || 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImFvdHlzeGN0d2J6ZHdha2JjbmdyIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NjUyMjEwNzEsImV4cCI6MjA4MDc5NzA3MX0._FrBhneIew2Rw61i5wczBzajqUqHimIGks9oAlgOcQ8';

export const supabase = createClient(SUPABASE_URL, SUPABASE_ANON_KEY);