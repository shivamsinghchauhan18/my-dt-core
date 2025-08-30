#!/usr/bin/env python3
"""
Comprehensive System Validation Runner

Main script to execute comprehensive validation of the Advanced Autonomous
Duckietown System against all requirements. This script integrates with the
existing easy_regression framework and provides detailed logging and monitoring.
"""

import os
import sys
import time
import argparse
import json
from pathlib import Path
from datetime import datetime
import logging

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(__file__))

from comprehensive_system_validation import ComprehensiveSystemValidator
from enhanced_performance_benchmarking import create_default_timing_requirements

# Configure comprehensive logging for validation execution
def setup_logging(log_level: str = "INFO", log_file: str = None):
    """Set up comprehensive logging for validation execution."""
    
    # Create formatter with timestamps
    formatter = logging.Formatter(
        '%(asctime)s [%(name)s] %(levelname)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # Set up root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(getattr(logging, log_level.upper()))
    
    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(formatter)
    root_logger.addHandler(console_handler)
    
    # File handler if specified
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(formatter)
        root_logger.addHandler(file_handler)
    
    return root_logger


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Comprehensive System Validation for Advanced Autonomous Duckietown System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run full validation with default settings
  python3 run_comprehensive_validation.py
  
  # Run validation with custom output directory
  python3 run_comprehensive_validation.py --output-dir /tmp/validation_results
  
  # Run only functional requirements validation
  python3 run_comprehensive_validation.py --functional-only
  
  # Run with verbose logging
  python3 run_comprehensive_validation.py --log-level DEBUG
  
  # Generate report only (skip validation)
  python3 run_comprehensive_validation.py --report-only --results-dir /path/to/results
        """
    )
    
    parser.add_argument(
        '--output-dir',
        type=str,
        default='validation_results',
        help='Output directory for validation results (default: validation_results)'
    )
    
    parser.add_argument(
        '--log-level',
        type=str,
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
        default='INFO',
        help='Logging level (default: INFO)'
    )
    
    parser.add_argument(
        '--log-file',
        type=str,
        help='Log file path (default: console only)'
    )
    
    parser.add_argument(
        '--functional-only',
        action='store_true',
        help='Run only functional requirements validation'
    )
    
    parser.add_argument(
        '--performance-only',
        action='store_true',
        help='Run only performance requirements validation'
    )
    
    parser.add_argument(
        '--safety-only',
        action='store_true',
        help='Run only safety requirements validation'
    )
    
    parser.add_argument(
        '--integration-only',
        action='store_true',
        help='Run only integration requirements validation'
    )
    
    parser.add_argument(
        '--report-only',
        action='store_true',
        help='Generate report only (skip validation)'
    )
    
    parser.add_argument(
        '--results-dir',
        type=str,
        help='Directory containing existing validation results (for report-only mode)'
    )
    
    parser.add_argument(
        '--requirements-file',
        type=str,
        default='.kiro/specs/advanced-autonomous-duckietown/requirements.md',
        help='Path to requirements document (default: .kiro/specs/advanced-autonomous-duckietown/requirements.md)'
    )
    
    parser.add_argument(
        '--timeout',
        type=int,
        default=3600,
        help='Validation timeout in seconds (default: 3600)'
    )
    
    parser.add_argument(
        '--continue-on-failure',
        action='store_true',
        help='Continue validation even if individual tests fail'
    )
    
    parser.add_argument(
        '--export-format',
        type=str,
        choices=['json', 'markdown', 'both'],
        default='both',
        help='Export format for validation report (default: both)'
    )
    
    return parser.parse_args()


def validate_environment():
    """Validate that the environment is set up correctly for validation."""
    logger = logging.getLogger(__name__)
    
    logger.info("Validating environment setup...")
    
    # Check if we're in the correct directory
    if not Path('packages').exists():
        logger.error("Not in dt-core root directory. Please run from dt-core root.")
        return False
    
    # Check for requirements file
    requirements_file = Path('.kiro/specs/advanced-autonomous-duckietown/requirements.md')
    if not requirements_file.exists():
        logger.warning(f"Requirements file not found: {requirements_file}")
        logger.warning("Some validation features may not work correctly")
    
    # Check for key packages
    key_packages = [
        'src/packages/duckietown_msgs',
        'src/packages/vehicle_detection',
        'src/packages/navigation',
        'src/packages/lane_control',
        'src/packages/fsm'
    ]
    
    missing_packages = []
    for package in key_packages:
        if not Path(package).exists():
            missing_packages.append(package)
    
    if missing_packages:
        logger.warning(f"Missing packages: {missing_packages}")
        logger.warning("Some validation tests may fail")
    
    # Check Python dependencies
    required_deps = ['numpy', 'yaml', 'psutil']
    missing_deps = []
    
    for dep in required_deps:
        try:
            __import__(dep)
        except ImportError:
            missing_deps.append(dep)
    
    if missing_deps:
        logger.error(f"Missing Python dependencies: {missing_deps}")
        logger.error("Please install missing dependencies before running validation")
        return False
    
    logger.info("Environment validation completed successfully")
    return True


def run_validation_step(validator, step_name, step_function, continue_on_failure=False):
    """Run a validation step with error handling and logging."""
    logger = logging.getLogger(__name__)
    
    logger.info(f"Starting validation step: {step_name}")
    step_start_time = time.time()
    
    try:
        result = step_function()
        step_duration = time.time() - step_start_time
        
        logger.info(f"Validation step '{step_name}' completed in {step_duration:.2f} seconds")
        return result, None
    
    except Exception as e:
        step_duration = time.time() - step_start_time
        error_msg = f"Validation step '{step_name}' failed after {step_duration:.2f} seconds: {e}"
        
        if continue_on_failure:
            logger.error(error_msg)
            logger.warning(f"Continuing validation despite failure in {step_name}")
            return None, str(e)
        else:
            logger.error(error_msg)
            raise


def main():
    """Main function to run comprehensive system validation."""
    
    # Parse arguments
    args = parse_arguments()
    
    # Set up logging
    log_file = args.log_file
    if log_file and not os.path.isabs(log_file):
        log_file = os.path.join(args.output_dir, log_file)
    
    logger = setup_logging(args.log_level, log_file)
    
    logger.info("="*80)
    logger.info("ADVANCED AUTONOMOUS DUCKIETOWN SYSTEM - COMPREHENSIVE VALIDATION")
    logger.info("="*80)
    logger.info(f"Validation started at: {datetime.now().isoformat()}")
    logger.info(f"Output directory: {args.output_dir}")
    logger.info(f"Log level: {args.log_level}")
    if log_file:
        logger.info(f"Log file: {log_file}")
    
    validation_start_time = time.time()
    
    try:
        # Validate environment
        if not validate_environment():
            logger.error("Environment validation failed. Exiting.")
            return 1
        
        # Create output directory
        output_dir = Path(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # Handle report-only mode
        if args.report_only:
            if not args.results_dir:
                logger.error("--results-dir must be specified for --report-only mode")
                return 1
            
            logger.info("Running in report-only mode")
            # TODO: Implement report generation from existing results
            logger.warning("Report-only mode not yet implemented")
            return 0
        
        # Create validator
        logger.info("Initializing comprehensive system validator...")
        validator = ComprehensiveSystemValidator(output_dir=str(output_dir))
        
        # Determine which validation steps to run
        run_functional = not any([args.performance_only, args.safety_only, args.integration_only])
        run_performance = not any([args.functional_only, args.safety_only, args.integration_only])
        run_safety = not any([args.functional_only, args.performance_only, args.integration_only])
        run_integration = not any([args.functional_only, args.performance_only, args.safety_only])
        
        if args.functional_only:
            run_functional = True
            run_performance = run_safety = run_integration = False
        elif args.performance_only:
            run_performance = True
            run_functional = run_safety = run_integration = False
        elif args.safety_only:
            run_safety = True
            run_functional = run_performance = run_integration = False
        elif args.integration_only:
            run_integration = True
            run_functional = run_performance = run_safety = False
        
        logger.info(f"Validation steps to run:")
        logger.info(f"  Functional Requirements: {'Yes' if run_functional else 'No'}")
        logger.info(f"  Performance Requirements: {'Yes' if run_performance else 'No'}")
        logger.info(f"  Safety Requirements: {'Yes' if run_safety else 'No'}")
        logger.info(f"  Integration Requirements: {'Yes' if run_integration else 'No'}")
        
        # Run validation steps
        validation_errors = []
        
        if run_functional:
            logger.info("="*60)
            logger.info("STEP 1: FUNCTIONAL REQUIREMENTS VALIDATION")
            logger.info("="*60)
            
            functional_results, error = run_validation_step(
                validator,
                "Functional Requirements",
                validator.validate_functional_requirements,
                args.continue_on_failure
            )
            
            if error:
                validation_errors.append(f"Functional validation: {error}")
        
        if run_performance:
            logger.info("="*60)
            logger.info("STEP 2: PERFORMANCE REQUIREMENTS VALIDATION")
            logger.info("="*60)
            
            performance_results, error = run_validation_step(
                validator,
                "Performance Requirements",
                validator.validate_performance_requirements,
                args.continue_on_failure
            )
            
            if error:
                validation_errors.append(f"Performance validation: {error}")
        
        if run_safety:
            logger.info("="*60)
            logger.info("STEP 3: SAFETY REQUIREMENTS VALIDATION")
            logger.info("="*60)
            
            safety_results, error = run_validation_step(
                validator,
                "Safety Requirements",
                validator.validate_safety_requirements,
                args.continue_on_failure
            )
            
            if error:
                validation_errors.append(f"Safety validation: {error}")
        
        if run_integration:
            logger.info("="*60)
            logger.info("STEP 4: INTEGRATION REQUIREMENTS VALIDATION")
            logger.info("="*60)
            
            integration_results, error = run_validation_step(
                validator,
                "Integration Requirements",
                validator.validate_integration_requirements,
                args.continue_on_failure
            )
            
            if error:
                validation_errors.append(f"Integration validation: {error}")
        
        # Generate final report
        logger.info("="*60)
        logger.info("GENERATING FINAL VALIDATION REPORT")
        logger.info("="*60)
        
        final_report, error = run_validation_step(
            validator,
            "Final Report Generation",
            validator.generate_final_validation_report,
            args.continue_on_failure
        )
        
        if error:
            validation_errors.append(f"Report generation: {error}")
        
        # Calculate total validation time
        total_validation_time = time.time() - validation_start_time
        
        # Print final summary
        logger.info("="*80)
        logger.info("VALIDATION SUMMARY")
        logger.info("="*80)
        logger.info(f"Total validation time: {total_validation_time:.2f} seconds")
        
        if validation_errors:
            logger.error(f"Validation completed with {len(validation_errors)} errors:")
            for error in validation_errors:
                logger.error(f"  - {error}")
        
        if final_report:
            logger.info(f"Overall result: {'PASSED' if final_report.overall_success else 'FAILED'}")
            logger.info(f"Compliance rate: {final_report.compliance_rate:.2%}")
            logger.info(f"Requirements validated: {final_report.validated_requirements}/{final_report.total_requirements}")
            
            # Print detailed results
            if final_report.requirement_results:
                functional_passed = len([r for r in final_report.requirement_results if r.success])
                logger.info(f"Functional tests: {functional_passed}/{len(final_report.requirement_results)} passed")
            
            if final_report.safety_results:
                safety_passed = len([r for r in final_report.safety_results if r.safety_score >= 0.8])
                logger.info(f"Safety tests: {safety_passed}/{len(final_report.safety_results)} passed")
            
            if final_report.performance_results:
                perf_success = final_report.performance_results.get('overall_performance_success', False)
                logger.info(f"Performance tests: {'PASSED' if perf_success else 'FAILED'}")
            
            if final_report.integration_results:
                int_success = final_report.integration_results.get('overall_success', False)
                logger.info(f"Integration tests: {'PASSED' if int_success else 'FAILED'}")
            
            # Print recommendations
            if final_report.recommendations:
                logger.info("Recommendations:")
                for i, rec in enumerate(final_report.recommendations, 1):
                    logger.info(f"  {i}. {rec}")
            
            # Determine exit code
            if final_report.overall_success and not validation_errors:
                logger.info("🎉 COMPREHENSIVE VALIDATION PASSED!")
                logger.info("System is ready for deployment.")
                return 0
            else:
                logger.warning("⚠️  COMPREHENSIVE VALIDATION FAILED!")
                logger.warning("Please address the identified issues before deployment.")
                return 1
        
        else:
            logger.error("Failed to generate final validation report")
            return 1
    
    except KeyboardInterrupt:
        logger.info("Validation interrupted by user")
        return 130
    
    except Exception as e:
        logger.error(f"Validation failed with unexpected error: {e}")
        logger.exception("Full traceback:")
        return 1
    
    finally:
        total_time = time.time() - validation_start_time
        logger.info("="*80)
        logger.info(f"Validation session ended at: {datetime.now().isoformat()}")
        logger.info(f"Total session time: {total_time:.2f} seconds")
        logger.info("="*80)


if __name__ == "__main__":
    sys.exit(main())