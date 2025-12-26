// Text selection handler using browser Selection API
const TextSelectionHandler = {
  // Get currently selected text
  getSelectedText: () => {
    if (typeof window !== 'undefined') {
      return window.getSelection ? window.getSelection().toString().trim() : '';
    }
    return '';
  },

  // Get detailed selection information
  getSelectionInfo: () => {
    if (typeof window !== 'undefined') {
      const selection = window.getSelection();
      if (selection.rangeCount > 0) {
        const range = selection.getRangeAt(0);
        const selectedText = selection.toString().trim();

        // Get surrounding context
        const startContainer = range.startContainer;
        const endContainer = range.endContainer;

        return {
          text: selectedText,
          range: range,
          startContainer: startContainer,
          endContainer: endContainer,
          rect: range.getBoundingClientRect(),
          isRange: range.toString().trim().length > 0
        };
      }
    }
    return null;
  },

  // Add event listeners for text selection
  addSelectionListener: (callback) => {
    if (typeof document !== 'undefined') {
      const handleSelection = () => {
        const selectionInfo = TextSelectionHandler.getSelectionInfo();
        if (selectionInfo && selectionInfo.isRange) {
          callback(selectionInfo);
        }
      };

      // Listen for selection changes
      document.addEventListener('mouseup', handleSelection);
      document.addEventListener('keyup', handleSelection);

      // For mobile devices
      document.addEventListener('touchend', handleSelection);

      // Return a function to remove the listeners
      return () => {
        document.removeEventListener('mouseup', handleSelection);
        document.removeEventListener('keyup', handleSelection);
        document.removeEventListener('touchend', handleSelection);
      };
    }
  },

  // Clear current selection
  clearSelection: () => {
    if (typeof window !== 'undefined') {
      if (window.getSelection) {
        window.getSelection().removeAllRanges();
      } else if (document.selection) {
        document.selection.empty();
      }
    }
  },

  // Validate selected text length
  validateSelection: (text) => {
    if (!text) return false;

    // Minimum length check
    if (text.length < 3) return false;

    // Maximum length check (adjust as needed)
    if (text.length > 1000) return false;

    return true;
  }
};

export default TextSelectionHandler;