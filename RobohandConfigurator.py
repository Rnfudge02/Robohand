#!/usr/bin/env python3
"""
@file RobohandConfigurator.py
@brief GUI Tool for configuring the RoboHand project
@details Provides a user-friendly interface to configure the RoboHand project
         by modifying preprocessor macros in header files.
@author Claude AI Assistant
@date 2025
@copyright Apache 2.0 License
"""

import os
import re
import sys
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

@dataclass
class MacroDefinition:
    """
    @brief Class to represent a preprocessor macro definition.
    @details Contains all the information about a macro including its value,
             file location, and metadata for the UI.
    """
    name: str                 # Macro name
    value: str                # Current value
    file_path: str            # Path to the file containing the macro
    line_number: int          # Line number in the file
    description: str = ""     # Description extracted from comments
    type: str = "bool"        # Data type (bool, int, string)
    category: str = "General" # UI category for organization

class RoboHandConfigurator:
    """
    @brief Main application class for the RoboHand Configurator.
    @details Provides a GUI for editing configuration macros in the RoboHand project.
    """
    
    def __init__(self, root):
        """
        @brief Initialize the configurator application.
        @param root Root Tkinter window
        """
        self.root = root
        self.root.title("RoboHand Configurator")
        self.root.geometry("800x600")
        self.root.minsize(800, 600)
        
        # Set up variables
        self.project_path = tk.StringVar()
        self.macros = []
        self.macro_var_map = {}  # Map from macro name to Tkinter variable
        
        # Set up the main UI layout
        self._setup_ui()
    
    def _setup_ui(self):
        """
        @brief Create the main UI layout.
        @details Sets up the frames, tabs, and controls for the application.
        """
        # Top frame for project selection
        top_frame = ttk.Frame(self.root, padding=10)
        top_frame.pack(fill=tk.X)
        
        ttk.Label(top_frame, text="Project Path:").pack(side=tk.LEFT, padx=(0, 5))
        ttk.Entry(top_frame, textvariable=self.project_path, width=50).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        ttk.Button(top_frame, text="Browse...", command=self._browse_project).pack(side=tk.LEFT)
        
        # Separator
        ttk.Separator(self.root, orient=tk.HORIZONTAL).pack(fill=tk.X, padx=10, pady=5)
        
        # Frame for configuration tabs
        self.tab_control = ttk.Notebook(self.root)
        self.tab_control.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Create tabs for different categories
        self.tabs = {
            "Hardware": ttk.Frame(self.tab_control),
            "Communication": ttk.Frame(self.tab_control),
            "I/O": ttk.Frame(self.tab_control),
            "General": ttk.Frame(self.tab_control)
        }
        
        # Map of category names to tabs for flexibility
        self.category_to_tab = {
            "Hardware": "Hardware",
            "Communication": "Communication", 
            "I/O": "I/O",
            "General": "General",
            # Add mappings for variations
            "Sensor": "Hardware",
            "Interface": "Communication"
        }
        
        for name, tab in self.tabs.items():
            self.tab_control.add(tab, text=name)
            
            # Add scrollable frame to each tab
            frame = ttk.Frame(tab)
            frame.pack(fill=tk.BOTH, expand=True)
            
            canvas = tk.Canvas(frame)
            scrollbar = ttk.Scrollbar(frame, orient="vertical", command=canvas.yview)
            scrollable_frame = ttk.Frame(canvas)
            
            scrollable_frame.bind(
                "<Configure>",
                lambda e, canvas=canvas: canvas.configure(scrollregion=canvas.bbox("all"))
            )
            
            canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
            canvas.configure(yscrollcommand=scrollbar.set)
            
            canvas.pack(side="left", fill="both", expand=True)
            scrollbar.pack(side="right", fill="y")
            
            # Store reference to scrollable frame
            self.tabs[name] = scrollable_frame
        
        # Bottom frame for action buttons
        bottom_frame = ttk.Frame(self.root, padding=10)
        bottom_frame.pack(fill=tk.X, side=tk.BOTTOM)
        
        ttk.Button(bottom_frame, text="Save Configuration", command=self._save_configuration).pack(side=tk.RIGHT, padx=5)
        ttk.Button(bottom_frame, text="Load Configuration", command=self._load_default_configuration).pack(side=tk.RIGHT, padx=5)
        ttk.Button(bottom_frame, text="Reset to Defaults", command=self._reset_to_defaults).pack(side=tk.RIGHT, padx=5)
    
    def _browse_project(self):
        """
        @brief Open a directory browser to select the project path.
        @details Shows a file dialog and initializes processing when a directory is selected.
        """
        directory = filedialog.askdirectory(title="Select RoboHand Project Directory")
        if directory:
            self.project_path.set(directory)
            self._load_project_files()
    
    def _load_project_files(self):
        """
        @brief Load and parse the project header files to extract macro definitions.
        @details Scans the Include directory for .h files and extracts configuration macros.
        """
        project_path = self.project_path.get()
        
        if not os.path.isdir(project_path):
            messagebox.showerror("Error", "The selected path is not a valid directory.")
            return
        
        # Clear existing macros
        self.macros = []
        
        # Look for header files in the Include directory
        include_dir = os.path.join(project_path, "Include")
        if not os.path.isdir(include_dir):
            messagebox.showerror("Error", "Cannot find 'Include' directory in the project path.")
            return
        
        # Parse each header file
        for filename in os.listdir(include_dir):
            if filename.endswith(".h"):
                file_path = os.path.join(include_dir, filename)
                self._parse_header_file(file_path)
        
        # Add manual categorization for specific macros
        for macro in self.macros:
            # Hardware-related macros
            if any(prefix in macro.name for prefix in ['HAS_', 'NUM_']):
                if any(hw in macro.name.lower() for hw in ['adc', 'bme', 'qmc', 'mpu', 'i2c', 'pi_']):
                    macro.category = "Hardware"
                elif any(io in macro.name.lower() for io in ['servo', 'rgb', 'led', 'gpio']):
                    macro.category = "I/O"
            
            # Communication-related macros
            if any(prefix in macro.name for prefix in ['USE_']):
                if any(comm in macro.name.lower() for comm in ['dma', 'interrupt', 'callback']):
                    macro.category = "Communication"
        
        # Create the UI for the found macros
        self._create_macro_ui()
    
    def _parse_header_file(self, file_path):
        """
        @brief Parse a header file to extract macro definitions.
        @details Searches for #define statements and extracts macro information.
        @param file_path Path to the header file to parse
        """
        try:
            with open(file_path, 'r') as file:
                lines = file.readlines()
                
            category = self._determine_category(file_path)
            
            # First pass: find all #define directives
            define_lines = []
            for i, line in enumerate(lines):
                # Look for #define statements
                define_match = re.match(r'^\s*#define\s+(\w+)\s+(.+)$', line)
                if define_match:
                    define_lines.append((i, line))
            
            # Second pass: look for comments before each #define
            for i, (line_num, line) in enumerate(define_lines):
                define_match = re.match(r'^\s*#define\s+(\w+)\s+(.+)$', line)
                if define_match:
                    name = define_match.group(1)
                    raw_value = define_match.group(2)
                    
                    # Skip internal or complex macros
                    if name.startswith('_') or '(' in name or '\\' in line:
                        continue
                    
                    # Extract value (remove any comments)
                    if '//' in raw_value:
                        value = raw_value.split('//', 1)[0].strip()
                    else:
                        value = raw_value.strip()
                    
                    # Extract description from comments (if available)
                    description = ""
                    
                    # Check for comment at the end of the line
                    if '//' in line:
                        description = line.split('//', 1)[1].strip()
                    
                    # If no comment on the same line, look for a comment block above
                    if not description and line_num > 0:
                        j = line_num - 1
                        while j >= 0 and (lines[j].strip().startswith('//') or not lines[j].strip()):
                            if lines[j].strip().startswith('//'):
                                comment_text = lines[j].strip()[2:].strip()
                                if comment_text:
                                    if description:
                                        description = comment_text + " " + description
                                    else:
                                        description = comment_text
                            j -= 1
                    
                    # Determine macro type
                    macro_type = "string"
                    if value.lower() in ('true', 'false'):
                        macro_type = "bool"
                    elif value.isdigit() or (value.startswith('-') and value[1:].isdigit()):
                        macro_type = "int"
                    
                    # Make a better guess at the category based on the macro name
                    macro_category = category
                    
                    # Override category based on macro name patterns
                    if any(name.startswith(prefix) for prefix in ['HAS_']):
                        if any(hw in name.lower() for hw in ['adc', 'i2c', 'bme', 'mpu', 'qmc', 'pi_']):
                            macro_category = "Hardware"
                        elif any(io in name.lower() for io in ['servo', 'rgb', 'led']):
                            macro_category = "I/O"
                    
                    if any(name.startswith(prefix) for prefix in ['USE_']):
                        if any(comm in name.lower() for comm in ['dma', 'interrupt', 'callback']):
                            macro_category = "Communication"
                    
                    # Add to our macros list
                    macro = MacroDefinition(
                        name=name,
                        value=value,
                        file_path=file_path,
                        line_number=line_num,
                        description=description,
                        type=macro_type,
                        category=macro_category
                    )
                    
                    # Only add if it looks like a configuration macro
                    if self._is_configuration_macro(macro):
                        self.macros.append(macro)
        
        except Exception as e:
            messagebox.showerror("Error", f"Error parsing file {os.path.basename(file_path)}: {str(e)}")
    
    def _determine_category(self, file_path):
        """
        @brief Determine the category of a header file based on its name or content.
        @param file_path Path to the header file
        @return Category string ('Hardware', 'I/O', 'Communication', or 'General')
        """
        filename = os.path.basename(file_path).lower()
        
        # Improved categorization based on the actual file structure and naming in your project
        if any(pattern in filename for pattern in ['i2c', 'adc', 'bme', 'mpu', 'qmc', 'reader']):
            return "Hardware"
        elif any(pattern in filename for pattern in ['servo', 'rgb']):
            return "I/O"
        elif any(pattern in filename for pattern in ['dma', 'interrupt', 'callback', 'uros', 'communication']):
            return "Communication"
        else:
            return "General"
    
    def _is_configuration_macro(self, macro):
        """
        @brief Determine if a macro is likely a configuration option.
        @param macro MacroDefinition object to check
        @return True if the macro should be included in the configuration UI
        """
        # Specific prefixes for configuration macros
        config_prefixes = ['HAS_', 'USE_', 'ENABLE_']
        
        # Check if the macro is a boolean type with a specific prefix
        if any(macro.name.startswith(prefix) for prefix in config_prefixes):
            return True
        
        # Check for known configuration macro names
        known_config_macros = [
            'NUM_SERVOS', 'DEBUG', 'SYS_CLOCK', 'ADC2_PIN', 'SERVO_MIN_PULSE', 
            'SERVO_MAX_PULSE', 'MAX_MOVE_DURATION_MS', 'SERVO_PWM_FREQ',
            'I2C_PORT', 'SDA_PIN', 'SCL_PIN', 'RGB_RED_PIN', 'RGB_GREEN_PIN', 'RGB_BLUE_PIN',
            'ADS1115_ADDR', 'BME280_ADDR', 'MPU6050_ADDR', 'QMC5883L_ADDR',
            'DMA_CHANNEL_I2C', 'DMA_CHANNEL_ADC', 'DMA_ADC_SAMPLES'
        ]
        
        if macro.name in known_config_macros:
            return True
            
        # Look for pin definitions and other hardware-specific configuration
        if macro.name.endswith('_PIN') or macro.name.endswith('_ADDR'):
            return True
            
        return False
    
    def _create_macro_ui(self):
        """
        @brief Create UI elements for each found macro.
        @details Generates the appropriate control widgets based on macro type.
        """
        # Clear existing UI
        for category, tab in self.tabs.items():
            for widget in tab.winfo_children():
                widget.destroy()
        
        self.macro_var_map = {}
        
        # Group macros by category
        grouped_macros = {}
        for macro in self.macros:
            if macro.category not in grouped_macros:
                grouped_macros[macro.category] = []
            grouped_macros[macro.category].append(macro)
        
        # Create UI elements for each macro by category
        for category, macros in grouped_macros.items():
            tab_name = self.category_to_tab.get(category, "General")
            tab = self.tabs.get(tab_name, self.tabs["General"])
            
            for i, macro in enumerate(macros):
                frame = ttk.Frame(tab, padding=5)
                frame.pack(fill=tk.X, padx=10, pady=2)
                
                # Create variable and widget based on macro type
                if macro.type == "bool":
                    var = tk.BooleanVar(value=macro.value.lower() == 'true')
                    widget = ttk.Checkbutton(frame, text=macro.name, variable=var)
                    widget.pack(side=tk.LEFT, fill=tk.X, expand=True)
                    
                elif macro.type == "int":
                    var = tk.IntVar(value=int(macro.value))
                    ttk.Label(frame, text=f"{macro.name}:").pack(side=tk.LEFT, padx=(0, 5))
                    ttk.Spinbox(frame, from_=0, to=1000000, textvariable=var, width=10).pack(side=tk.LEFT, padx=(0, 5))
                    
                else:  # string type
                    var = tk.StringVar(value=macro.value)
                    ttk.Label(frame, text=f"{macro.name}:").pack(side=tk.LEFT, padx=(0, 5))
                    ttk.Entry(frame, textvariable=var, width=30).pack(side=tk.LEFT, padx=(0, 5))
                
                # Store variable reference
                self.macro_var_map[macro.name] = var
                
                # Add description as tooltip if available
                if macro.description:
                    desc_label = ttk.Label(frame, text="?", foreground="blue")
                    desc_label.pack(side=tk.LEFT, padx=5)
                    self._create_tooltip(desc_label, macro.description)
                
                # Add file info
                file_label = ttk.Label(frame, text=os.path.basename(macro.file_path), foreground="blue")
                file_label.pack(side=tk.RIGHT)
                
                # Add separator except for the last item
                if i < len(macros) - 1:
                    ttk.Separator(tab, orient=tk.HORIZONTAL).pack(fill=tk.X, padx=15, pady=2)
    
    def _create_tooltip(self, widget, text):
        """
        @brief Create a tooltip that appears when hovering over a widget.
        @param widget The widget to attach the tooltip to
        @param text The text to display in the tooltip
        """
        def enter(event):
            x = y = 0
            x, y, _, _ = widget.bbox("insert")
            x += widget.winfo_rootx() + 25
            y += widget.winfo_rooty() + 25
            
            # Create a toplevel window
            self.tooltip = tk.Toplevel(widget)
            self.tooltip.wm_overrideredirect(True)
            self.tooltip.wm_geometry(f"+{x}+{y}")
            
            label = ttk.Label(self.tooltip, text=text, justify=tk.LEFT,
                             background="#ffffe0", relief=tk.SOLID, borderwidth=1,
                             padding=5)
            label.pack(ipadx=1)
        
        def leave(event):
            if hasattr(self, 'tooltip'):
                self.tooltip.destroy()
        
        widget.bind("<Enter>", enter)
        widget.bind("<Leave>", leave)
    
    def _create_backup(self, file_path):
        """
        @brief Create a timestamped backup of a file before modifying it.
        @details Copies the original file to a backup with timestamp in the filename.
        @param file_path Path to the file to back up
        @return Path to the backup file
        """
        import datetime
        import shutil
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_dir = os.path.join(os.path.dirname(file_path), "backups")
        
        # Create backup directory if it doesn't exist
        if not os.path.exists(backup_dir):
            os.makedirs(backup_dir)
            
        # Generate backup filename with timestamp
        filename = os.path.basename(file_path)
        backup_path = os.path.join(backup_dir, f"{os.path.splitext(filename)[0]}_{timestamp}.h")
        
        # Create the backup
        shutil.copy2(file_path, backup_path)
        
        return backup_path
    
    def _save_configuration(self):
        """
        @brief Save the configuration changes back to the header files.
        @details Updates header files with the new macro values from the UI.
        Creates timestamped backups of modified files before changes are applied.
        """
        if not self.macros:
            messagebox.showinfo("Info", "No configuration loaded.")
            return
        
        # Group macros by file
        files_to_update = {}
        for macro in self.macros:
            if macro.file_path not in files_to_update:
                files_to_update[macro.file_path] = []
            
            # Get current value from UI
            var = self.macro_var_map.get(macro.name)
            if var:
                new_value = var.get()
                
                # Format value based on type
                if isinstance(new_value, bool):
                    formatted_value = "true" if new_value else "false"
                else:
                    formatted_value = str(new_value)
                
                # Add to update list if changed
                if formatted_value != macro.value:
                    files_to_update[macro.file_path].append((macro, formatted_value))
        
        # Process each file
        updated_files = 0
        backups = []
        
        for file_path, updates in files_to_update.items():
            if not updates:
                continue
            
            try:
                # Create backup before modifying
                backup_path = self._create_backup(file_path)
                backups.append(backup_path)
                
                # Read file
                with open(file_path, 'r') as file:
                    lines = file.readlines()
                
                # Apply updates (in reverse order to avoid line number changes)
                for macro, new_value in sorted(updates, key=lambda x: x[0].line_number, reverse=True):
                    line = lines[macro.line_number]
                    
                    # Find the position of the existing value
                    define_match = re.match(r'^\s*#define\s+(\w+)\s+(.+)$', line)
                    if define_match:
                        # Get the part of the line after the macro name
                        after_name = define_match.group(2)
                        
                        # Find where the value ends (at a comment or end of line)
                        if '//' in after_name:
                            value_part, comment_part = after_name.split('//', 1)
                            # Replace just the value part and keep the comment
                            new_line = f'#define {macro.name} {new_value} //{comment_part}'
                        else:
                            # No comment, just replace the value
                            new_line = f'#define {macro.name} {new_value}'
                        
                        lines[macro.line_number] = new_line
                
                # Write back to file
                with open(file_path, 'w') as file:
                    file.writelines(lines)
                
                updated_files += 1
                
            except Exception as e:
                messagebox.showerror("Error", f"Error updating file {os.path.basename(file_path)}: {str(e)}")
        
        if updated_files > 0:
            backup_msg = f"\n\nBackups created in the 'backups' directory:\n" + "\n".join([os.path.basename(b) for b in backups])
            messagebox.showinfo("Success", f"Configuration saved to {updated_files} file(s).{backup_msg}")
        else:
            messagebox.showinfo("Info", "No changes were made.")
    
    def _load_default_configuration(self):
        """
        @brief Load the current configuration from the files.
        @details Re-scans the project files to update the UI with current values.
        """
        if self.project_path.get():
            self._load_project_files()
        else:
            messagebox.showinfo("Info", "Please select a project directory first.")
    
    def _reset_to_defaults(self):
        """
        @brief Reset all configuration options to their original values.
        @details Resets the UI controls to match the values in the source files.
        """
        if not self.macros:
            return
            
        # Confirm reset
        if not messagebox.askyesno("Confirm Reset", "Reset all settings to their original values?"):
            return
            
        # Reset each macro to its original value
        for macro in self.macros:
            var = self.macro_var_map.get(macro.name)
            if var:
                if macro.type == "bool":
                    var.set(macro.value.lower() == 'true')
                elif macro.type == "int":
                    var.set(int(macro.value))
                else:
                    var.set(macro.value)

def main():
    """
    @brief Main entry point for the application.
    @details Creates and launches the Tkinter application.
    """
    root = tk.Tk()
    app = RoboHandConfigurator(root)
    root.mainloop()

if __name__ == "__main__":
    main()