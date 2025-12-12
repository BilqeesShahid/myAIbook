#!/usr/bin/env python3
"""
Script to validate the setup of the RAG system.
Checks if all required components are properly configured.
"""

import sys
import os
import logging
from typing import Dict, List

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def validate_environment_variables() -> Dict[str, bool]:
    """
    Validate that required environment variables are set.
    """
    logger.info("Validating environment variables...")

    results = {}

    # Check required environment variables
    required_vars = [
        'QDRANT_URL',
        'QDRANT_API_KEY',
        'GOOGLE_API_KEY'
    ]

    for var in required_vars:
        value = os.getenv(var)
        is_set = value is not None and len(value) > 0
        results[var] = is_set
        if not is_set:
            logger.error(f"Environment variable {var} is not set")
        else:
            logger.info(f"Environment variable {var} is set")

    return results


def validate_qdrant_connection() -> bool:
    """
    Validate that we can connect to Qdrant.
    """
    logger.info("Validating Qdrant connection...")

    try:
        from qdrant_client import QdrantClient
        import backend.src.utils.constants as constants

        client = QdrantClient(
            url=constants.QDRANT_URL,
            api_key=constants.QDRANT_API_KEY,
            prefer_grpc=False
        )

        # Try to get collection info to test connection
        try:
            client.get_collection(constants.QDRANT_COLLECTION_NAME)
            logger.info("Successfully connected to Qdrant and verified collection exists")
            return True
        except:
            # If collection doesn't exist, try to create it as a test
            try:
                client.create_collection(
                    collection_name=constants.QDRANT_COLLECTION_NAME,
                    vectors_config={
                        "size": constants.EMBEDDING_DIMENSION,
                        "distance": "Cosine"
                    }
                )
                logger.info("Successfully connected to Qdrant and created collection")
                return True
            except Exception as e:
                logger.error(f"Failed to connect to Qdrant: {e}")
                return False

    except ImportError:
        logger.error("Qdrant client not installed")
        return False
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {e}")
        return False


def validate_google_api() -> bool:
    """
    Validate that Google API key is working.
    """
    logger.info("Validating Google API connection...")

    try:
        import google.generativeai as genai
        import backend.src.utils.constants as constants

        genai.configure(api_key=constants.GOOGLE_API_KEY)

        # Test by generating a simple embedding
        test_text = "This is a test for API validation"
        result = genai.embed_content(
            model="models/embedding-001",
            content=[test_text],
            task_type="retrieval_document"
        )

        if result and 'embedding' in result and len(result['embedding']) > 0:
            logger.info("Successfully validated Google API connection")
            return True
        else:
            logger.error("Google API validation failed - no embedding returned")
            return False

    except ImportError:
        logger.error("Google Generative AI not installed")
        return False
    except Exception as e:
        logger.error(f"Failed to validate Google API: {e}")
        return False


def validate_docs_folder() -> Dict[str, any]:
    """
    Validate that the docs folder exists and contains markdown files.
    """
    logger.info("Validating docs folder...")

    docs_path = os.path.join(os.path.dirname(__file__), "..", "..", "docs")

    if not os.path.exists(docs_path):
        logger.error(f"Docs folder does not exist at {docs_path}")
        return {"exists": False, "file_count": 0, "has_markdown": False}

    # Count markdown files
    md_files = []
    for root, dirs, files in os.walk(docs_path):
        for file in files:
            if file.lower().endswith('.md'):
                md_files.append(os.path.join(root, file))

    has_markdown = len(md_files) > 0

    result = {
        "exists": True,
        "path": docs_path,
        "file_count": len(md_files),
        "has_markdown": has_markdown,
        "files": md_files[:5]  # Show first 5 files as sample
    }

    if has_markdown:
        logger.info(f"Found {len(md_files)} markdown files in docs folder")
    else:
        logger.warning("No markdown files found in docs folder")

    return result


def validate_dependencies() -> Dict[str, bool]:
    """
    Validate that required dependencies are installed.
    """
    logger.info("Validating dependencies...")

    required_packages = [
        'fastapi',
        'uvicorn',
        'qdrant_client',
        'google_generativeai',
        'pydantic',
        'python-dotenv',
        'markdown'
    ]

    results = {}

    for package in required_packages:
        try:
            if package == 'google_generativeai':
                # Special case for google-generativeai
                __import__('google.generativeai')
            elif package == 'qdrant_client':
                # Special case for qdrant-client
                __import__('qdrant_client')
            else:
                __import__(package)
            results[package] = True
            logger.info(f"Dependency {package} is available")
        except ImportError:
            results[package] = False
            logger.error(f"Dependency {package} is not available")

    return results


def run_full_validation() -> Dict[str, any]:
    """
    Run full validation of the setup.
    """
    logger.info("Starting full setup validation...")

    results = {
        "timestamp": __import__('datetime').datetime.now().isoformat(),
        "environment": validate_environment_variables(),
        "qdrant_connection": validate_qdrant_connection(),
        "google_api": validate_google_api(),
        "docs_folder": validate_docs_folder(),
        "dependencies": validate_dependencies()
    }

    # Overall status
    all_env_vars_set = all(results["environment"].values())
    all_deps_available = all(results["dependencies"].values())

    results["overall_status"] = (
        all_env_vars_set and
        results["qdrant_connection"] and
        results["google_api"] and
        results["docs_folder"]["has_markdown"] and
        all_deps_available
    )

    logger.info(f"Overall validation status: {'PASS' if results['overall_status'] else 'FAIL'}")

    return results


if __name__ == "__main__":
    results = run_full_validation()

    print("\n" + "="*50)
    print("SETUP VALIDATION RESULTS")
    print("="*50)

    print(f"Timestamp: {results['timestamp']}")
    print(f"Overall Status: {'PASS' ✅' if results['overall_status'] else 'FAIL ❌'}")

    print("\nEnvironment Variables:")
    for var, status in results['environment'].items():
        print(f"  {var}: {'✅' if status else '❌'}")

    print(f"\nQdrant Connection: {'✅' if results['qdrant_connection'] else '❌'}")
    print(f"Google API: {'✅' if results['google_api'] else '❌'}")

    print(f"\nDocs Folder:")
    print(f"  Exists: {'✅' if results['docs_folder']['exists'] else '❌'}")
    print(f"  Has Markdown Files: {'✅' if results['docs_folder']['has_markdown'] else '❌'}")
    if results['docs_folder']['has_markdown']:
        print(f"  Count: {results['docs_folder']['file_count']}")

    print(f"\nDependencies:")
    for dep, status in results['dependencies'].items():
        print(f"  {dep}: {'✅' if status else '❌'}")

    if not results['overall_status']:
        sys.exit(1)
    else:
        print(f"\n✅ All validations passed!")
        sys.exit(0)