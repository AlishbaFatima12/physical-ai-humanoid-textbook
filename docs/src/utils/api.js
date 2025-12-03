/**
 * Smart Backend URL Detection
 *
 * Priority:
 * 1. Check if local backend is running (localhost:8000)
 * 2. Fall back to production Render backend
 */

// Production backend URL
const PRODUCTION_URL = 'https://physical-ai-humanoid-textbook.onrender.com';

// Local backend URL
const LOCAL_URL = 'http://localhost:8000';

let cachedBackendUrl = null;
let checkPromise = null;

/**
 * Check if local backend is accessible
 */
async function isLocalBackendAvailable() {
  try {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), 1000); // 1 second timeout

    const response = await fetch(`${LOCAL_URL}/health`, {
      method: 'GET',
      signal: controller.signal,
    });

    clearTimeout(timeoutId);
    return response.ok;
  } catch (error) {
    return false;
  }
}

/**
 * Get the appropriate backend URL
 * Caches the result to avoid multiple checks
 */
export async function getBackendUrl() {
  // Return cached URL if available
  if (cachedBackendUrl) {
    return cachedBackendUrl;
  }

  // If a check is already in progress, wait for it
  if (checkPromise) {
    return checkPromise;
  }

  // Start new check
  checkPromise = (async () => {
    const isLocalAvailable = await isLocalBackendAvailable();

    if (isLocalAvailable) {
      console.log('✅ Using local backend:', LOCAL_URL);
      cachedBackendUrl = LOCAL_URL;
    } else {
      console.log('☁️ Using production backend:', PRODUCTION_URL);
      cachedBackendUrl = PRODUCTION_URL;
    }

    checkPromise = null;
    return cachedBackendUrl;
  })();

  return checkPromise;
}

/**
 * Make an API request to the backend
 */
export async function apiRequest(endpoint, options = {}) {
  const baseUrl = await getBackendUrl();
  const url = `${baseUrl}${endpoint}`;

  const defaultHeaders = {
    'Content-Type': 'application/json',
  };

  // Add auth token if available
  const authToken = localStorage.getItem('authToken');
  if (authToken) {
    defaultHeaders['Authorization'] = `Bearer ${authToken}`;
  }

  const response = await fetch(url, {
    ...options,
    headers: {
      ...defaultHeaders,
      ...options.headers,
    },
  });

  if (!response.ok) {
    throw new Error(`API request failed: ${response.statusText}`);
  }

  return response.json();
}

/**
 * Force refresh of the backend URL (useful for testing)
 */
export function refreshBackendUrl() {
  cachedBackendUrl = null;
  checkPromise = null;
}
