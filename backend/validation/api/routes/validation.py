"""
Validation API routes for the retrieval validation service.

This module defines all the API endpoints for validation functionality.
"""
from fastapi import APIRouter, HTTPException, BackgroundTasks
from pydantic import BaseModel
from typing import List, Dict, Optional, Any
import uuid
import asyncio
from datetime import datetime
from ...services.vector_loader import vector_loader_service
from ...services.metadata_inspector import metadata_inspector_service
from ...query_executor import query_execution_engine
from ...test_queries import TestQueryManager, default_query_manager
from ...search import similarity_search_service
from ...relevance_scoring import relevance_scoring_service
from ...metadata_validator import metadata_validator_service
from ...validation_analyzer import validation_result_analyzer
from ...comprehensive_report import comprehensive_report_service
from ...persistence import persistence_service
from ...base import TestQuery, RetrievedResult


router = APIRouter()


# Pydantic models for request/response
class TestQueryRequest(BaseModel):
    id: str
    text: str
    category: Optional[str] = None
    expected_result_ids: Optional[List[str]] = None


class ValidationRunRequest(BaseModel):
    test_queries: List[TestQueryRequest]
    top_k: int = 5
    collection_name: Optional[str] = None


class ValidationRunResponse(BaseModel):
    validation_id: str
    status: str
    total_queries: int
    completed_at: Optional[str]
    metrics: Optional[Dict[str, float]]


class TestQueryExecutionRequest(BaseModel):
    query_text: str
    top_k: int = 5
    collection_name: Optional[str] = None


class TestQueryExecutionResponse(BaseModel):
    query_text: str
    retrieved_results: List[Dict[str, Any]]
    retrieval_time_ms: float


class ValidationStatusResponse(BaseModel):
    validation_id: str
    status: str
    created_at: str
    completed_at: Optional[str]
    metrics: Optional[Dict[str, float]]
    detailed_results: Optional[List[Dict[str, Any]]]


class OverallMetricsResponse(BaseModel):
    overall_precision: float
    overall_metadata_accuracy: float
    total_validations_run: int
    avg_retrieval_time_ms: float
    last_validation_at: Optional[str]


# In-memory storage for validation run status (in production, use a database)
validation_runs_status: Dict[str, Dict[str, Any]] = {}


@router.post("/run", response_model=ValidationRunResponse)
async def run_validation(request: ValidationRunRequest, background_tasks: BackgroundTasks) -> ValidationRunResponse:
    """
    Trigger a comprehensive retrieval validation run.
    """
    validation_id = str(uuid.uuid4())

    # Initialize run status
    validation_runs_status[validation_id] = {
        "status": "running",
        "created_at": datetime.now().isoformat(),
        "total_queries": len(request.test_queries),
        "completed_at": None,
        "metrics": None,
        "detailed_results": None
    }

    # Convert request queries to internal TestQuery objects
    test_queries = []
    for q in request.test_queries:
        test_query = TestQuery(
            id=q.id,
            text=q.text,
            category=q.category,
            expected_result_ids=q.expected_result_ids
        )
        test_queries.append(test_query)

    # Run validation in background
    background_tasks.add_task(
        _execute_validation_run,
        validation_id,
        test_queries,
        request.top_k,
        request.collection_name
    )

    return ValidationRunResponse(
        validation_id=validation_id,
        status="running",
        total_queries=len(request.test_queries),
        completed_at=None,
        metrics=None
    )


async def _execute_validation_run(validation_id: str, test_queries: List[TestQuery],
                                top_k: int, collection_name: Optional[str]):
    """
    Execute the validation run in the background.
    """
    try:
        # Update status to running
        validation_runs_status[validation_id]["status"] = "running"

        # Execute queries
        results = []
        for test_query in test_queries:
            result = query_execution_engine.execute_single_query(test_query, top_k, collection_name)
            results.append(result)

        # Calculate metrics
        total_precision = sum(r.metrics.precision_at_k for r in results if hasattr(r.metrics, 'precision_at_k')) / len(results) if results else 0.0
        total_metadata_accuracy = sum(r.metrics.metadata_accuracy for r in results if hasattr(r.metrics, 'metadata_accuracy')) / len(results) if results else 0.0

        # Prepare detailed results
        detailed_results = []
        for i, result in enumerate(results):
            formatted_result = query_execution_engine.format_query_result(result)
            detailed_results.append(formatted_result)

        # Create a validation run object for persistence
        from ...models import ValidationRun
        validation_run = ValidationRun(
            id=validation_id,
            test_queries=test_queries,
            results=[],  # This would be the validation results model
            overall_metrics={
                "precision_at_k": total_precision,
                "metadata_accuracy": total_metadata_accuracy
            },
            start_time=datetime.fromisoformat(validation_runs_status[validation_id]["created_at"]),
            end_time=datetime.now(),
            completed=True
        )

        # Save to persistence
        persistence_service.save_validation_run(validation_run)

        # Update status to completed
        validation_runs_status[validation_id].update({
            "status": "completed",
            "completed_at": datetime.now().isoformat(),
            "metrics": {
                "precision_at_k": total_precision,
                "metadata_accuracy": total_metadata_accuracy
            },
            "detailed_results": detailed_results
        })

    except Exception as e:
        # Update status to failed
        validation_runs_status[validation_id].update({
            "status": "failed",
            "completed_at": datetime.now().isoformat(),
            "error": str(e)
        })
        raise


@router.get("/{validation_id}", response_model=ValidationStatusResponse)
async def get_validation_status(validation_id: str) -> ValidationStatusResponse:
    """
    Get the status and results of a validation run.
    """
    if validation_id not in validation_runs_status:
        raise HTTPException(status_code=404, detail="Validation run not found")

    run_data = validation_runs_status[validation_id]

    return ValidationStatusResponse(
        validation_id=validation_id,
        status=run_data["status"],
        created_at=run_data["created_at"],
        completed_at=run_data.get("completed_at"),
        metrics=run_data.get("metrics"),
        detailed_results=run_data.get("detailed_results")
    )


@router.post("/test-query", response_model=TestQueryExecutionResponse)
async def test_single_query(request: TestQueryExecutionRequest) -> TestQueryExecutionResponse:
    """
    Test a single query against the retrieval system.
    """
    try:
        # Create a temporary test query
        test_query = TestQuery(
            id=str(uuid.uuid4()),
            text=request.query_text
        )

        # Execute the query
        result = query_execution_engine.execute_single_query(test_query, request.top_k, request.collection_name)

        # Format results
        formatted_results = query_execution_engine.format_results_for_display(result.retrieved_results)

        return TestQueryExecutionResponse(
            query_text=request.query_text,
            retrieved_results=formatted_results,
            retrieval_time_ms=result.execution_time_ms
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error executing query: {str(e)}")


@router.get("/metrics", response_model=OverallMetricsResponse)
async def get_overall_metrics() -> OverallMetricsResponse:
    """
    Get overall validation metrics across all runs.
    """
    # Get all validation runs from persistence
    runs_info = persistence_service.list_validation_runs()

    if not runs_info:
        return OverallMetricsResponse(
            overall_precision=0.0,
            overall_metadata_accuracy=0.0,
            total_validations_run=0,
            avg_retrieval_time_ms=0.0,
            last_validation_at=None
        )

    # Calculate overall metrics from persisted runs
    total_precision = 0.0
    total_metadata_accuracy = 0.0
    total_retrieval_time = 0.0
    count = 0

    for run_info in runs_info:
        run_summary = persistence_service.get_validation_run_summary(run_info['id'])
        if run_summary:
            total_precision += run_summary.get('avg_precision', 0.0)
            total_metadata_accuracy += run_summary.get('avg_metadata_accuracy', 0.0)
            total_retrieval_time += run_summary.get('avg_retrieval_time_ms', 0.0)
            count += 1

    avg_precision = total_precision / count if count > 0 else 0.0
    avg_metadata_accuracy = total_metadata_accuracy / count if count > 0 else 0.0
    avg_retrieval_time = total_retrieval_time / count if count > 0 else 0.0

    last_run = runs_info[0] if runs_info else None
    last_validation_at = last_run['modified_at'] if last_run else None

    return OverallMetricsResponse(
        overall_precision=avg_precision,
        overall_metadata_accuracy=avg_metadata_accuracy,
        total_validations_run=len(runs_info),
        avg_retrieval_time_ms=avg_retrieval_time,
        last_validation_at=last_validation_at
    )


# Additional utility endpoints
@router.get("/runs")
async def list_validation_runs():
    """
    List all validation runs.
    """
    return persistence_service.list_validation_runs()


@router.delete("/runs/{run_id}")
async def delete_validation_run(run_id: str) -> Dict[str, str]:
    """
    Delete a validation run.
    """
    success = persistence_service.delete_validation_run(run_id)
    if not success:
        raise HTTPException(status_code=404, detail="Validation run not found")

    return {"message": f"Validation run {run_id} deleted successfully"}