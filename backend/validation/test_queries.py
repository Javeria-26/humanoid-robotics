"""
Test query management system for the retrieval validation system.

This module provides functionality to manage and organize test queries
for validation purposes.
"""
import json
import uuid
from datetime import datetime
from typing import List, Dict, Optional, Any
from .base import TestQuery
from .logger import validation_logger as logger


class TestQueryManager:
    """
    Manager class for handling test queries used in validation.
    """

    def __init__(self):
        self.queries: Dict[str, TestQuery] = {}
        self.categories: Dict[str, List[str]] = {}  # category -> list of query IDs

    def add_query(self, query: TestQuery) -> bool:
        """
        Add a test query to the manager.

        Args:
            query: TestQuery object to add

        Returns:
            bool: True if successfully added, False otherwise
        """
        try:
            self.queries[query.id] = query

            # Add to category index if category exists
            if query.category:
                if query.category not in self.categories:
                    self.categories[query.category] = []
                if query.id not in self.categories[query.category]:
                    self.categories[query.category].append(query.id)

            logger.info(f"Added test query: {query.id} - {query.text[:50]}...")
            return True
        except Exception as e:
            logger.error(f"Error adding test query: {str(e)}")
            return False

    def create_query(self, text: str, category: Optional[str] = None,
                     expected_result_ids: Optional[List[str]] = None) -> Optional[TestQuery]:
        """
        Create and add a new test query.

        Args:
            text: The query text
            category: Optional category for the query
            expected_result_ids: Optional list of expected relevant result IDs

        Returns:
            Created TestQuery object or None if creation failed
        """
        try:
            query_id = str(uuid.uuid4())
            test_query = TestQuery(
                id=query_id,
                text=text,
                category=category,
                expected_result_ids=expected_result_ids
            )

            if self.add_query(test_query):
                return test_query
            else:
                return None
        except Exception as e:
            logger.error(f"Error creating test query: {str(e)}")
            return None

    def get_query(self, query_id: str) -> Optional[TestQuery]:
        """
        Get a test query by its ID.

        Args:
            query_id: ID of the query to retrieve

        Returns:
            TestQuery object or None if not found
        """
        return self.queries.get(query_id)

    def get_queries_by_category(self, category: str) -> List[TestQuery]:
        """
        Get all test queries in a specific category.

        Args:
            category: Category to filter by

        Returns:
            List of TestQuery objects in the category
        """
        query_ids = self.categories.get(category, [])
        return [self.queries[qid] for qid in query_ids if qid in self.queries]

    def get_all_queries(self) -> List[TestQuery]:
        """
        Get all test queries.

        Returns:
            List of all TestQuery objects
        """
        return list(self.queries.values())

    def remove_query(self, query_id: str) -> bool:
        """
        Remove a test query by its ID.

        Args:
            query_id: ID of the query to remove

        Returns:
            bool: True if successfully removed, False otherwise
        """
        if query_id not in self.queries:
            logger.warning(f"Attempted to remove non-existent query: {query_id}")
            return False

        query = self.queries[query_id]

        # Remove from category index
        if query.category and query.category in self.categories:
            if query_id in self.categories[query.category]:
                self.categories[query.category].remove(query_id)

        del self.queries[query_id]
        logger.info(f"Removed test query: {query_id}")
        return True

    def get_categories(self) -> List[str]:
        """
        Get all available categories.

        Returns:
            List of category names
        """
        return list(self.categories.keys())

    def load_from_dict(self, data: Dict[str, Any]) -> bool:
        """
        Load test queries from a dictionary.

        Args:
            data: Dictionary containing query data

        Returns:
            bool: True if successfully loaded, False otherwise
        """
        try:
            queries_data = data.get('queries', [])
            for query_data in queries_data:
                query = TestQuery(
                    id=query_data['id'],
                    text=query_data['text'],
                    category=query_data.get('category'),
                    expected_result_ids=query_data.get('expected_result_ids'),
                    created_at=datetime.fromisoformat(query_data['created_at']) if query_data.get('created_at') else datetime.now()
                )
                self.add_query(query)

            logger.info(f"Loaded {len(queries_data)} test queries from dictionary")
            return True
        except Exception as e:
            logger.error(f"Error loading test queries from dictionary: {str(e)}")
            return False

    def load_from_json_file(self, filepath: str) -> bool:
        """
        Load test queries from a JSON file.

        Args:
            filepath: Path to the JSON file

        Returns:
            bool: True if successfully loaded, False otherwise
        """
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
            return self.load_from_dict(data)
        except Exception as e:
            logger.error(f"Error loading test queries from file {filepath}: {str(e)}")
            return False

    def save_to_dict(self) -> Dict[str, Any]:
        """
        Save test queries to a dictionary.

        Returns:
            Dictionary containing query data
        """
        queries_list = []
        for query in self.queries.values():
            query_dict = {
                'id': query.id,
                'text': query.text,
                'category': query.category,
                'expected_result_ids': query.expected_result_ids,
                'created_at': query.created_at.isoformat() if query.created_at else None
            }
            queries_list.append(query_dict)

        return {
            'queries': queries_list,
            'total_count': len(queries_list),
            'categories': list(self.categories.keys()),
            'exported_at': datetime.now().isoformat()
        }

    def save_to_json_file(self, filepath: str) -> bool:
        """
        Save test queries to a JSON file.

        Args:
            filepath: Path to save the JSON file

        Returns:
            bool: True if successfully saved, False otherwise
        """
        try:
            data = self.save_to_dict()
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            logger.info(f"Saved {len(self.queries)} test queries to {filepath}")
            return True
        except Exception as e:
            logger.error(f"Error saving test queries to file {filepath}: {str(e)}")
            return False

    def get_query_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about the test queries.

        Returns:
            Dictionary with query statistics
        """
        total_queries = len(self.queries)
        queries_by_category = {cat: len(ids) for cat, ids in self.categories.items()}
        total_categories = len(self.categories)

        return {
            'total_queries': total_queries,
            'total_categories': total_categories,
            'queries_by_category': queries_by_category,
            'has_expected_results': sum(1 for q in self.queries.values() if q.expected_result_ids),
            'timestamp': datetime.now().isoformat()
        }


# Predefined test queries for common validation scenarios
PREDEFINED_QUERIES = [
    {
        "text": "What is humanoid robotics?",
        "category": "robotics_concepts"
    },
    {
        "text": "Explain inverse kinematics",
        "category": "robotics_concepts"
    },
    {
        "text": "How does the perception system work?",
        "category": "robotics_systems"
    },
    {
        "text": "What is ROS 2?",
        "category": "software_tools"
    },
    {
        "text": "Describe the control architecture",
        "category": "robotics_systems"
    },
    {
        "text": "How is the robot calibrated?",
        "category": "robotics_processes"
    },
    {
        "text": "What sensors are used in the system?",
        "category": "robotics_hardware"
    },
    {
        "text": "Explain the path planning algorithm",
        "category": "robotics_algorithms"
    }
]


def create_default_query_manager() -> TestQueryManager:
    """
    Create a TestQueryManager with default test queries.

    Returns:
        TestQueryManager with predefined queries loaded
    """
    manager = TestQueryManager()

    for query_data in PREDEFINED_QUERIES:
        manager.create_query(
            text=query_data["text"],
            category=query_data["category"]
        )

    logger.info(f"Created default query manager with {len(PREDEFINED_QUERIES)} predefined queries")
    return manager


# Global instance for convenience
default_query_manager = create_default_query_manager()