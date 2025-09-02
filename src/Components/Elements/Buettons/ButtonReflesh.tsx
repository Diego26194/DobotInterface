import { MenuItem } from "@mui/material";
import RefreshIcon from '@mui/icons-material/Refresh';

interface ButtonRefleshProps {
  onClick: () => void;
}

const ButtonReflesh = ({ onClick }: ButtonRefleshProps) => {
  return (
    <MenuItem onClick={onClick}>
      <RefreshIcon fontSize="small" style={{ marginRight: 2 }} color="primary"/>
    </MenuItem>
  );
};

export default ButtonReflesh;