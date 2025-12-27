// API service for backend communication
// Use environment variable or default to production URL
const DEFAULT_API_URL = typeof window !== 'undefined'
  ? `${window.location.protocol}//${window.location.hostname}:${window.location.port || 8000}/api/v1`
  : 'http://localhost:8000/api/v1';

const API_BASE_URL = (typeof process !== 'undefined' && process.env && process.env.REACT_APP_BACKEND_API_URL)
  ? process.env.REACT_APP_BACKEND_API_URL
  : DEFAULT_API_URL;

class ApiService {
  constructor() {
    this.baseURL = API_BASE_URL;
    // Initialize cache for recent queries
    this.queryCache = new Map();
    this.maxCacheSize = 50; // Maximum number of cached items
  }

  // Helper method to make API requests with enhanced CORS and error handling
  async makeRequest(endpoint, options = {}) {
    const url = `${this.baseURL}${endpoint}`;
    const defaultOptions = {
      headers: {
        'Content-Type': 'application/json',
        // Add any default headers here
      },
      // Enable CORS mode explicitly
      mode: 'cors',
      // Include credentials if needed for auth
      credentials: 'omit', // Change to 'include' if backend requires cookies/sessions
    };

    // Merge default options with provided options
    const requestConfig = {
      ...defaultOptions,
      ...options,
      headers: {
        ...defaultOptions.headers,
        ...options.headers,
      },
    };

    try {
      const response = await fetch(url, requestConfig);

      // Check if response is ok (status 200-299)
      if (!response.ok) {
        // Handle different error types
        if (response.status === 401 || response.status === 403) {
          throw new Error(`Authentication failed: ${response.status} - ${response.statusText}`);
        } else if (response.status === 404) {
          throw new Error('API endpoint not found. Please check your backend connection.');
        } else if (response.status >= 500) {
          throw new Error(`Server error: ${response.status} - ${response.statusText}. The backend service may be unavailable.`);
        } else {
          const errorData = await response.json().catch(() => ({}));
          throw new Error(errorData.message || `HTTP error! status: ${response.status}`);
        }
      }

      return await response.json();
    } catch (error) {
      // Handle network errors, CORS errors, and other fetch-related errors
      if (error.name === 'TypeError' && error.message.includes('fetch')) {
        throw new Error('Network error: Unable to connect to the server. Please check if the backend is running and accessible.');
      } else if (error.message.includes('CORS')) {
        throw new Error('CORS error: Cross-Origin Resource Sharing is blocking the request. Please check backend CORS configuration.');
      } else if (error.message.includes('Failed to fetch')) {
        throw new Error('Connection error: Cannot reach the backend server. Please verify the backend URL and ensure the server is running.');
      }
      console.error('API request failed:', error);
      throw error;
    }
  }

  // Method to submit a query to the backend
  async submitQuery(query, context = {}, sessionId = null) {
    const payload = {
      query: query,
      context: context,
      ...(sessionId && { session_id: sessionId }),
    };

    return this.makeRequest('/query', {
      method: 'POST',
      body: JSON.stringify(payload),
    });
  }

  // Method to check backend health
  async healthCheck() {
    return this.makeRequest('/health', {
      method: 'GET',
    });
  }

  // Method to validate query before sending
  validateQuery(query) {
    if (!query || typeof query !== 'string' || query.trim().length === 0) {
      throw new Error('Query must be a non-empty string');
    }

    // Check for query length limits (adjust as needed)
    if (query.length > 1000) {
      throw new Error('Query exceeds maximum length of 1000 characters');
    }

    return true;
  }

  // Method to implement timeout for API requests
  async submitQueryWithTimeout(query, context = {}, sessionId = null, timeout = 10000) {
    // Create a timeout promise
    const timeoutPromise = new Promise((_, reject) => {
      setTimeout(() => {
        reject(new Error('Request timeout after ' + timeout + 'ms'));
      }, timeout);
    });

    // Make the actual API request
    const requestPromise = this.submitQuery(query, context, sessionId);

    // Race between the request and the timeout
    return Promise.race([requestPromise, timeoutPromise]);
  }

  // Method to implement caching for recent queries/responses
  cacheQuery(query, context, response) {
    // Create a cache key from query and context
    const cacheKey = JSON.stringify({ query, context });

    // Add to cache
    this.queryCache.set(cacheKey, {
      response,
      timestamp: Date.now()
    });

    // Remove oldest items if cache is too large
    if (this.queryCache.size > this.maxCacheSize) {
      const firstKey = this.queryCache.keys().next().value;
      this.queryCache.delete(firstKey);
    }
  }

  // Method to retrieve cached response
  getCachedResponse(query, context) {
    const cacheKey = JSON.stringify({ query, context });
    const cached = this.queryCache.get(cacheKey);

    // Check if cache is still valid (less than 5 minutes old)
    if (cached && Date.now() - cached.timestamp < 5 * 60 * 1000) {
      return cached.response;
    }

    // Remove expired cache entry
    if (cached) {
      this.queryCache.delete(cacheKey);
    }

    return null;
  }

  // Method to clear the cache
  clearCache() {
    this.queryCache.clear();
  }

  // Enhanced query method with caching
  async submitQueryWithCache(query, context = {}, sessionId = null) {
    // Check cache first
    const cachedResponse = this.getCachedResponse(query, context);
    if (cachedResponse) {
      return cachedResponse;
    }

    // If not in cache, make the request
    const response = await this.submitQueryWithTimeout(query, context, sessionId);

    // Cache the response
    this.cacheQuery(query, context, response);

    return response;
  }

  // Enhanced error handling
  async submitQueryWithErrorHandling(query, context = {}, sessionId = null) {
    try {
      return await this.submitQueryWithCache(query, context, sessionId);
    } catch (error) {
      // Handle different types of errors
      if (error.message.includes('CORS error')) {
        throw new Error('Connection blocked by CORS policy. Please check backend CORS configuration.');
      } else if (error.message.includes('Connection error')) {
        throw new Error('Cannot connect to the backend server. Please verify the server is running and accessible.');
      } else if (error.message.includes('timeout')) {
        throw new Error('Request timeout: The server is taking too long to respond. Please try again later.');
      } else if (error.message.includes('Authentication failed')) {
        throw new Error('Authentication error: Unable to access the service. Please check your credentials.');
      } else if (error.message.includes('API endpoint not found')) {
        throw new Error('Service unavailable: The API endpoint cannot be found. Please check if the backend service is running.');
      } else if (error.message.includes('Server error')) {
        throw new Error('Server error: The service is temporarily unavailable. Please try again later.');
      } else if (error.name === 'TypeError' && error.message.includes('fetch')) {
        throw new Error('Network error: Unable to connect to the server. Please check your internet connection.');
      } else if (error.message.includes('HTTP error')) {
        if (error.message.includes('400')) {
          throw new Error('Invalid request: Please check your query and try again.');
        } else {
          throw new Error(`Service error: ${error.message}`);
        }
      } else {
        throw error;
      }
    }
  }
}

// Export a singleton instance
const apiService = new ApiService();
export default apiService;